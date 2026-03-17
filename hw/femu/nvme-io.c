#include "./nvme.h"

static uint16_t nvme_io_cmd(FemuCtrl *n, NvmeCmd *cmd, NvmeRequest *req);

static void nvme_update_sq_eventidx(const NvmeSQueue *sq)
{
    if (sq->eventidx_addr_hva)
    {
        *((uint32_t *)(sq->eventidx_addr_hva)) = sq->tail;
        return;
    }

    if (sq->eventidx_addr)
    {
        nvme_addr_write(sq->ctrl, sq->eventidx_addr, (void *)&sq->tail,
                        sizeof(sq->tail));
    }
}

static inline void nvme_copy_cmd(NvmeCmd *dst, NvmeCmd *src)
{
#if defined(__AVX__)
    __m256i *d256 = (__m256i *)dst;
    const __m256i *s256 = (const __m256i *)src;

    _mm256_store_si256(&d256[0], _mm256_load_si256(&s256[0]));
    _mm256_store_si256(&d256[1], _mm256_load_si256(&s256[1]));
#elif defined(__SSE2__)
    __m128i *d128 = (__m128i *)dst;
    const __m128i *s128 = (const __m128i *)src;

    _mm_store_si128(&d128[0], _mm_load_si128(&s128[0]));
    _mm_store_si128(&d128[1], _mm_load_si128(&s128[1]));
    _mm_store_si128(&d128[2], _mm_load_si128(&s128[2]));
    _mm_store_si128(&d128[3], _mm_load_si128(&s128[3]));
#else
    *dst = *src;
#endif
}

/**
 * 处理SQ的IO指令
 * @opaque: 传入的参数，实际是NvmeSQueue指针
 * @index_poller: 当前poller线程的索引，用于区分多线程下的不同to_ftl队列
 */
static void nvme_process_sq_io(void *opaque, int index_poller)
{
    NvmeSQueue *sq = opaque;
    FemuCtrl *n = sq->ctrl;

    uint16_t status;
    hwaddr addr;       // 存储指令在虚拟机物理内存的地址
    NvmeCmd cmd;       // 局部变量，用于存储从虚拟机内存中抓取的64字节SQE
    NvmeRequest *req;  // 指向FEMU内部的请求结构体，用于后续处理和记录请求状态
    int processed = 0; // 记录本轮轮询处理的指令条数

    // 更新尾指针，检查虚拟机是否写入了新的Doorbell，从而更新控制器内存中的tail
    nvme_update_sq_tail(sq);
    while (!(nvme_sq_empty(sq)))
    {
        // 提取指令（SQE）地址，支持物理连续和非连续两种情况

        if (sq->phys_contig)
        {
            // 情况A：队列在物理内存中是连续的
            // 计算指令地址：队列基地址 + (head索引 * SQE大小) 其中sq->dma_addr是虚拟机的物理地址（gpa），sq->dma_addr_hva是FEMU映射到宿主机的虚拟地址（hva）
            addr = sq->dma_addr + sq->head * n->sqe_size;
            // 直接从宿主机映射的虚拟地址复制64字节
            nvme_copy_cmd(&cmd, (void *)&(((NvmeCmd *)sq->dma_addr_hva)[sq->head]));
        }
        else
        {
            // 情况B：队列在物理内存中是非连续的，使用PRP列表进行地址转换，具体暂不注释
            addr = nvme_discontig(sq->prp_list, sq->head, n->page_size,
                                  n->sqe_size);
            nvme_addr_read(n, addr, (void *)&cmd, sizeof(cmd));
        }
        // 更新头指针，准备处理下一个指令
        nvme_inc_sq_head(sq);

        // 内部req对象初始化
        // 从预分配的请求空闲链表中取出一个托盘
        req = QTAILQ_FIRST(&sq->req_list);
        QTAILQ_REMOVE(&sq->req_list, req, entry);

        // 重置请求状态，清除之前的旧数据
        memset(&req->cqe, 0, sizeof(req->cqe));
        req->dsm_ranges = NULL;
        req->dsm_nr_ranges = 0;
        req->dsm_attributes = 0;

        // 记录关键时间戳，用于后续的时延模拟
        // stime：记录请求开始被处理的时间点（纳秒）
        // expire_time：初始等于开始时间，后续ftl会在此基础上加上模拟的闪存延迟
        /* Coperd: record req->stime at earliest convenience */
        req->expire_time = req->stime = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

        // 绑定id和操作码
        req->cqe.cid = cmd.cid;       // 命令id，完工时回传给虚拟机
        req->cmd_opcode = cmd.opcode; // 操作码，记录操作类型
        memcpy(&req->cmd, &cmd, sizeof(NvmeCmd));

        if (n->print_log)
        {
            femu_debug("%s,cid:%d\n", __func__, cmd.cid);
        }
        // 分发指令进行初步处理，对于不同类型的ssd，绑定了不同的处理函数，在nvme_register_extensions中设置
        status = nvme_io_cmd(n, &cmd, req);

        if (status == NVME_SUCCESS)
        {
            req->status = status;
            // 将处理好的请求放入到对应的to_ftl队列中，等待后续的时延模拟和完成处理
            // 具体处理逻辑可以看ftl_thread函数
            int rc = femu_ring_enqueue(n->to_ftl[index_poller], (void *)&req, 1);
            if (rc != 1)
            {
                // 如果队列满了，需紧急清理资源防止内存泄漏
                femu_err("enqueue failed, ret=%d\n", rc);
                // Clean up DSM ranges on enqueue failure
                if (req->dsm_ranges)
                {
                    g_free(req->dsm_ranges);
                    req->dsm_ranges = NULL;
                    req->dsm_nr_ranges = 0;
                }
            }
        }
        else
        {
            femu_err("Error IO processed! opcode=0x%x, status=0x%x\n",
                     cmd.opcode, status);
            req->status = status;

            // Clean up DSM ranges on error
            if (req->dsm_ranges)
            {
                g_free(req->dsm_ranges);
                req->dsm_ranges = NULL;
                req->dsm_nr_ranges = 0;
            }
        }

        processed++;
    }

    nvme_update_sq_eventidx(sq);
    sq->completed += processed;
}

static void nvme_post_cqe(NvmeCQueue *cq, NvmeRequest *req)
{
    FemuCtrl *n = cq->ctrl;
    NvmeSQueue *sq = req->sq;
    NvmeCqe *cqe = &req->cqe;
    uint8_t phase = cq->phase;
    hwaddr addr;

    if (n->print_log)
    {
        femu_debug("%s,req,lba:%lu,lat:%lu\n", n->devname, req->slba, req->reqlat);
    }
    cqe->status = cpu_to_le16((req->status << 1) | phase);
    cqe->sq_id = cpu_to_le16(sq->sqid);
    cqe->sq_head = cpu_to_le16(sq->head);

    if (cq->phys_contig)
    {
        addr = cq->dma_addr + cq->tail * n->cqe_size;
        ((NvmeCqe *)cq->dma_addr_hva)[cq->tail] = *cqe;
    }
    else
    {
        addr = nvme_discontig(cq->prp_list, cq->tail, n->page_size, n->cqe_size);
        nvme_addr_write(n, addr, (void *)cqe, sizeof(*cqe));
    }

    nvme_inc_cq_tail(cq);
}

/**
 * 推进CQ并处理已完成请求的函数
 * 原理：检查ftl传回的请求，对比当前时间与请求的expire_time
 * 只有到点的请求才会正式结束并向guest发送中断
 */
static void nvme_process_cq_cpl(void *arg, int index_poller)
{
    FemuCtrl *n = (FemuCtrl *)arg;
    NvmeCQueue *cq = NULL;
    NvmeRequest *req = NULL;

    // rp:默认指向to_ftl队列，但如果是BBSSD/ZNSSD则指向to_poller队列
    struct rte_ring *rp = n->to_ftl[index_poller];

    // pq:按时间排序的一个优先队列，因为后到的请求可能比先到的请求更快完成，pq保证我们总是先检查最快完成的那个
    pqueue_t *pq = n->pq[index_poller];
    uint64_t now;
    int processed = 0;
    int rc;
    int i;

    // 如果是BBSSD/ZNSSD，数据是从ftl计算完延迟之后通过to_poller队列送回来的
    if (BBSSD(n) || ZNSSD(n))
    {
        rp = n->to_poller[index_poller];
    }

    // 收割已完成的请求并存入优先队列
    while (femu_ring_count(rp))
    {
        req = NULL;
        rc = femu_ring_dequeue(rp, (void *)&req, 1);
        if (rc != 1)
        {
            femu_err("dequeue from to_poller request failed\n");
        }
        assert(req);

        pqueue_insert(pq, req);
    }

    while ((req = pqueue_peek(pq)))
    {
        now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

        // 如果当前时间还没到请求的到期时间，说明硬件“还没忙完”。
        // 因为 pq 是排序的，如果最快的一个都没到期，后面的肯定也没到期，直接跳出循环。
        if (now < req->expire_time)
        {
            break;
        }

        cq = n->cq[req->sq->sqid];
        if (!cq->is_active)
            continue;

        // 填充 CQE (Completion Queue Entry) 并通知虚拟机
        nvme_post_cqe(cq, req);

        // 将 req 放回 submission queue 的空闲列表中，供下次 IO 使用。
        QTAILQ_INSERT_TAIL(&req->sq->req_list, req, entry);
        pqueue_pop(pq);
        processed++;
        n->nr_tt_ios++;

        // 原理：理想情况下 now 应该等于 expire_time。
        // 如果 diff > 20,000ns (20微秒)，说明 FEMU 的轮询线程太忙了，处理晚了。
        if (now - req->expire_time >= 20000)
        {
            n->nr_tt_late_ios++;
            if (n->print_log)
            {
                femu_debug("%s,diff,pq.count=%lu,%" PRId64 ", %lu/%lu\n",
                           n->devname, pqueue_size(pq), now - req->expire_time,
                           n->nr_tt_late_ios, n->nr_tt_ios);
            }
        }
        n->should_isr[req->sq->sqid] = true;
    }

    if (processed == 0)
        return;

    switch (n->multipoller_enabled)
    {
    case 1:
        nvme_isr_notify_io(n->cq[index_poller]);
        break;
    default:
        for (i = 1; i <= n->nr_io_queues; i++)
        {
            if (n->should_isr[i])
            {
                nvme_isr_notify_io(n->cq[i]);
                n->should_isr[i] = false;
            }
        }
        break;
    }
}

/**
 * NVMe poller线程函数
 * 功能：不断检查虚拟机是否提交了SQ，并检查后端是否返回了CQ
 */
void *nvme_poller(void *arg)
{
    // 从参数中获得控制器对象和当前poller线程的索引
    FemuCtrl *n = ((NvmePollerThreadArgument *)arg)->n;
    int index = ((NvmePollerThreadArgument *)arg)->index;
    int i;

    /**
     * 根据是否开启了多轮询器模式进入不同的处理逻辑
     * 如果开启，每个poller线程只处理对应索引的一组SQ/CQ
     * 如果不开启，一个poller线程要处理所有的SQ/CQ
     */
    switch (n->multipoller_enabled)
    {
    case 1:
        while (1)
        {
            // 如果虚拟机还没完全启动或NVMe还没初始化好，就先睡眠一段时间，防止空转
            if ((!n->dataplane_started))
            {
                usleep(1000);
                continue;
            }

            // 在多线程模式下,Poller[index]只负责SQ[index]和CQ[index]
            NvmeSQueue *sq = n->sq[index];
            NvmeCQueue *cq = n->cq[index];

            // 队列激活并有新指令的情况下，处理SQ的IO请求
            if (sq && sq->is_active && cq && cq->is_active)
            {
                nvme_process_sq_io(sq, index);
            }

            // 看FTL线程有没有已经完成的任务
            nvme_process_cq_cpl(n, index);
        }
        break;
    default:
        while (1)
        {
            if ((!n->dataplane_started))
            {
                usleep(1000);
                continue;
            }
            // 遍历所有IO队列，依次每个队列是否有新指令需要处理
            for (i = 1; i <= n->nr_io_queues; i++)
            {
                NvmeSQueue *sq = n->sq[i];
                NvmeCQueue *cq = n->cq[i];
                if (sq && sq->is_active && cq && cq->is_active)
                {
                    // 所有的SQ请求都在这个index线程里处理，可能会有性能瓶颈，但实现简单
                    nvme_process_sq_io(sq, index);
                }
            }
            nvme_process_cq_cpl(n, index);
        }
        break;
    }

    return NULL;
}

/**
 *  处理读写指令的函数
 *  @n：当前控制器对象
 *  @ns：当前命名空间对象
 *  @cmd：从虚拟机内存中抓取的指令数据，
 *  @req：FEMU内部的请求对象，用于记录请求状态和后续处理
 */
uint16_t nvme_rw(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd, NvmeRequest *req)
{
    // 1.将通用命令结构体转换为读写命令专用的结构体
    NvmeRwCmd *rw = (NvmeRwCmd *)cmd;

    // 2.提取并转换字节序（从虚拟机小端格式转为主机cpu格式）
    uint16_t ctrl = le16_to_cpu(rw->control);
    uint32_t nlb = le16_to_cpu(rw->nlb) + 1; // 本次IO要读写的逻辑块数量，NVMe协议中是0-based，所以要加1
    uint64_t slba = le64_to_cpu(rw->slba);   // 起始ssd逻辑块地址
    uint64_t prp1 = le64_to_cpu(rw->prp1);   // 物理区域页指针1，指向guest内存存放数据的第一个4KB物理页
    uint64_t prp2 = le64_to_cpu(rw->prp2);   // 物理区域页指针2或PRP列表指针

    // 从命名空间获取lba格式索引，根据这个索引获取每个lba的大小（ms）
    const uint8_t lba_index = NVME_ID_NS_FLBAS_INDEX(ns->id_ns.flbas);
    // lba后面的元数据大小
    const uint16_t ms = le16_to_cpu(ns->id_ns.lbaf[lba_index].ms);
    // 逻辑块的大小的位数
    const uint8_t data_shift = ns->id_ns.lbaf[lba_index].lbads;
    // 本次IO设计的总数据长度（单位：字节），例如如果nlb=7，data_shift=12，那么data_size=7*4096=28672字节
    uint64_t data_size = (uint64_t)nlb << data_shift;
    // 起始地址在ssd中绝对字节位置
    uint64_t data_offset = slba << data_shift;
    // 元数据总大小
    uint64_t meta_size = nlb * ms;
    uint64_t elba = slba + nlb;
    uint16_t err;
    int ret;

    req->is_write = (rw->opcode == NVME_CMD_WRITE) ? 1 : 0;

    // 检查请求是否合法，包括防止越界、对其检查等内容
    err = femu_nvme_rw_check_req(n, ns, cmd, req, slba, elba, nlb, ctrl,
                                 data_size, meta_size);
    if (err)
        return err;

    // 将虚拟机提供的PRP地址转换为femu可以直接使用的sglist结构，方便后续直接进行DMA读写
    if (nvme_map_prp(&req->qsg, &req->iov, prp1, prp2, data_size, n))
    {
        nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_INVALID_FIELD,
                            offsetof(NvmeRwCmd, prp1), 0, ns->id);
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    assert((nlb << data_shift) == req->qsg.size);

    req->slba = slba;
    req->status = NVME_SUCCESS;
    req->nlb = nlb;

    ret = backend_rw(n->mbe, &req->qsg, &data_offset, req->is_write);
    if (!ret)
    {
        return NVME_SUCCESS;
    }

    return NVME_DNR;
}

static uint16_t nvme_dsm(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                         NvmeRequest *req)
{
    uint32_t cdw10 = le32_to_cpu(cmd->cdw10);
    uint32_t cdw11 = le32_to_cpu(cmd->cdw11);
    uint64_t prp1 = le64_to_cpu(cmd->dptr.prp1);
    uint64_t prp2 = le64_to_cpu(cmd->dptr.prp2);
    uint16_t nr_ranges;
    NvmeDsmRange *ranges = NULL;
    int i;

    // Extract number of ranges from CDW10 (bits 7:0, 0-based)
    nr_ranges = (cdw10 & 0xFF) + 1;

    // Validate range count - NVMe supports up to 256 ranges
    if (nr_ranges > 256)
    {
        femu_err("DSM: Invalid range count %u (max 256)\n", nr_ranges);
        nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_INVALID_FIELD,
                            offsetof(NvmeCmd, cdw10), nr_ranges, ns->id);
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    // Check if any deallocate operation is requested
    bool has_deallocate = (cdw11 & NVME_DSMGMT_AD) != 0;
    // bool has_idr = (cdw11 & NVME_DSMGMT_IDR) != 0;
    // bool has_idw = (cdw11 & NVME_DSMGMT_IDW) != 0;

    // femu_debug("DSM: nr_ranges=%u, AD=%d, IDR=%d, IDW=%d\n",
    //        nr_ranges, has_deallocate, has_idr, has_idw);

    // If no deallocate attribute, no need to process further for TRIM
    if (!has_deallocate)
    {
        femu_err("DSM: No deallocate attribute set, skipping\n");
        return NVME_SUCCESS;
    }

    // Allocate buffer for DSM ranges
    size_t ranges_size = sizeof(NvmeDsmRange) * nr_ranges;
    ranges = g_malloc0(ranges_size);
    if (!ranges)
    {
        femu_err("DSM: Failed to allocate memory for %u ranges\n", nr_ranges);
        return NVME_INTERNAL_DEV_ERROR | NVME_DNR;
    }

    if (dma_write_prp(n, (uint8_t *)ranges, ranges_size, prp1, prp2))
    {
        nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_INVALID_FIELD,
                            offsetof(NvmeCmd, dptr.prp1), 0, ns->id);
        g_free(ranges);
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    // Validate and process each range
    uint64_t total_blocks = 0;
    uint64_t ns_size = le64_to_cpu(ns->id_ns.nsze);

    for (i = 0; i < nr_ranges; i++)
    {
        uint64_t slba = le64_to_cpu(ranges[i].slba);
        uint32_t nlb = le32_to_cpu(ranges[i].nlb);
        // uint32_t cattr = le32_to_cpu(ranges[i].cattr);

        // femu_debug("    DSM Range %d: slba=%lu, nlb=%u, cattr=0x%x\n",
        //        i, slba, nlb, cattr);

        // Validate LBA range against namespace size
        if (slba >= ns_size)
        {
            femu_err("DSM: Range %d SLBA %lu exceeds namespace size %lu\n",
                     i, slba, ns_size);
            nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_LBA_RANGE,
                                offsetof(NvmeCmd, cdw10), slba, ns->id);
            g_free(ranges);
            return NVME_LBA_RANGE | NVME_DNR;
        }

        if (slba + nlb > ns_size)
        {
            femu_err("DSM: Range %d end LBA %lu exceeds namespace size %lu\n",
                     i, slba + nlb, ns_size);
            nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_LBA_RANGE,
                                offsetof(NvmeCmd, cdw10), slba + nlb, ns->id);
            g_free(ranges);
            return NVME_LBA_RANGE | NVME_DNR;
        }

        // Check for integer overflow in block count accumulation
        if (total_blocks > UINT64_MAX - nlb)
        {
            femu_err("DSM: Total block count overflow\n");
            g_free(ranges);
            return NVME_INVALID_FIELD | NVME_DNR;
        }

        total_blocks += nlb;

        // Update namespace utilization bitmap for this range
        if (ns->util)
        {
            bitmap_clear(ns->util, slba, nlb);
        }
    }

    femu_debug("DSM: Total blocks to deallocate: %lu\n", total_blocks);

    // Store ranges in request for FTL processing
    req->dsm_ranges = ranges;
    req->dsm_nr_ranges = nr_ranges;
    req->dsm_attributes = cdw11;
    req->cmd_opcode = NVME_CMD_DSM;

    // Don't free ranges here - FTL will handle them
    req->status = NVME_SUCCESS;
    return NVME_SUCCESS;
}

static uint16_t nvme_compare(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                             NvmeRequest *req)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)cmd;
    uint32_t nlb = le16_to_cpu(rw->nlb) + 1;
    uint64_t slba = le64_to_cpu(rw->slba);
    uint64_t prp1 = le64_to_cpu(rw->prp1);
    uint64_t prp2 = le64_to_cpu(rw->prp2);
    int i;

    uint64_t elba = slba + nlb;
    uint8_t lba_index = NVME_ID_NS_FLBAS_INDEX(ns->id_ns.flbas);
    uint8_t data_shift = ns->id_ns.lbaf[lba_index].lbads;
    uint64_t data_size = nlb << data_shift;
    uint64_t offset = ns->start_block + (slba << data_shift);

    if ((slba + nlb) > le64_to_cpu(ns->id_ns.nsze))
    {
        nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_LBA_RANGE,
                            offsetof(NvmeRwCmd, nlb), elba, ns->id);
        return NVME_LBA_RANGE | NVME_DNR;
    }
    if (n->id_ctrl.mdts && data_size > n->page_size * (1 << n->id_ctrl.mdts))
    {
        nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_INVALID_FIELD,
                            offsetof(NvmeRwCmd, nlb), nlb, ns->id);
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    if (nvme_map_prp(&req->qsg, &req->iov, prp1, prp2, data_size, n))
    {
        nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_INVALID_FIELD,
                            offsetof(NvmeRwCmd, prp1), 0, ns->id);
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    if (find_next_bit(ns->uncorrectable, elba, slba) < elba)
    {
        return NVME_UNRECOVERED_READ;
    }

    for (i = 0; i < req->qsg.nsg; i++)
    {
        uint32_t len = req->qsg.sg[i].len;
        uint8_t *tmp[2];

        tmp[0] = g_malloc0(len);
        tmp[1] = g_malloc0(len);

        nvme_addr_read(n, req->qsg.sg[i].base, tmp[1], len);
        if (memcmp(tmp[0], tmp[1], len))
        {
            qemu_sglist_destroy(&req->qsg);
            return NVME_CMP_FAILURE;
        }
        offset += len;
        g_free(tmp[0]);
        g_free(tmp[1]);
    }

    qemu_sglist_destroy(&req->qsg);

    return NVME_SUCCESS;
}

static uint16_t nvme_flush(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                           NvmeRequest *req)
{
    return NVME_SUCCESS;
}

static uint16_t nvme_write_zeros(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                                 NvmeRequest *req)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)cmd;
    uint64_t slba = le64_to_cpu(rw->slba);
    uint32_t nlb = le16_to_cpu(rw->nlb) + 1;

    if ((slba + nlb) > ns->id_ns.nsze)
    {
        nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_LBA_RANGE,
                            offsetof(NvmeRwCmd, nlb), slba + nlb, ns->id);
        return NVME_LBA_RANGE | NVME_DNR;
    }

    return NVME_SUCCESS;
}

static uint16_t nvme_write_uncor(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                                 NvmeRequest *req)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)cmd;
    uint64_t slba = le64_to_cpu(rw->slba);
    uint32_t nlb = le16_to_cpu(rw->nlb) + 1;

    if ((slba + nlb) > ns->id_ns.nsze)
    {
        nvme_set_error_page(n, req->sq->sqid, cmd->cid, NVME_LBA_RANGE,
                            offsetof(NvmeRwCmd, nlb), slba + nlb, ns->id);
        return NVME_LBA_RANGE | NVME_DNR;
    }

    bitmap_set(ns->uncorrectable, slba, nlb);

    return NVME_SUCCESS;
}

static uint16_t nvme_io_cmd(FemuCtrl *n, NvmeCmd *cmd, NvmeRequest *req)
{
    NvmeNamespace *ns;
    uint32_t nsid = le32_to_cpu(cmd->nsid);

    if (nsid == 0 || nsid > n->num_namespaces)
    {
        femu_err("%s, NVME_INVALID_NSID %" PRIu32 "\n", __func__, nsid);
        return NVME_INVALID_NSID | NVME_DNR;
    }

    req->ns = ns = &n->namespaces[nsid - 1];

    switch (cmd->opcode)
    {
    case NVME_CMD_FLUSH:
        if (!n->id_ctrl.vwc || !n->features.volatile_wc)
        {
            return NVME_SUCCESS;
        }
        return nvme_flush(n, ns, cmd, req);
    case NVME_CMD_DSM:
        if (NVME_ONCS_DSM & n->oncs)
        {
            return nvme_dsm(n, ns, cmd, req);
        }
        return NVME_INVALID_OPCODE | NVME_DNR;
    case NVME_CMD_COMPARE:
        if (NVME_ONCS_COMPARE & n->oncs)
        {
            return nvme_compare(n, ns, cmd, req);
        }
        return NVME_INVALID_OPCODE | NVME_DNR;
    case NVME_CMD_WRITE_ZEROES:
        if (NVME_ONCS_WRITE_ZEROS & n->oncs)
        {
            return nvme_write_zeros(n, ns, cmd, req);
        }
        return NVME_INVALID_OPCODE | NVME_DNR;
    case NVME_CMD_WRITE_UNCOR:
        if (NVME_ONCS_WRITE_UNCORR & n->oncs)
        {
            return nvme_write_uncor(n, ns, cmd, req);
        }
        return NVME_INVALID_OPCODE | NVME_DNR;
    default:
        if (n->ext_ops.io_cmd)
        {
            return n->ext_ops.io_cmd(n, ns, cmd, req);
        }

        femu_err("%s, NVME_INVALID_OPCODE\n", __func__);
        return NVME_INVALID_OPCODE | NVME_DNR;
    }
}

void nvme_post_cqes_io(void *opaque)
{
    NvmeCQueue *cq = opaque;
    NvmeRequest *req, *next;
    int64_t cur_time, ntt = 0;
    int processed = 0;

    QTAILQ_FOREACH_SAFE(req, &cq->req_list, entry, next)
    {
        if (nvme_cq_full(cq))
        {
            break;
        }

        cur_time = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
        if (cq->cqid != 0 && cur_time < req->expire_time)
        {
            ntt = req->expire_time;
            break;
        }

        nvme_post_cqe(cq, req);
        processed++;
    }

    if (ntt == 0)
    {
        ntt = qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + CQ_POLLING_PERIOD_NS;
    }

    /* Only interrupt guest when we "do" complete some I/Os */
    if (processed > 0)
    {
        nvme_isr_notify_io(cq);
    }
}
