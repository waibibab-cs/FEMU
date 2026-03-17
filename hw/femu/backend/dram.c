#include "../nvme.h"

/* Coperd: FEMU Memory Backend (mbe) for emulated SSD */

int init_dram_backend(SsdDramBackend **mbe, int64_t nbytes)
{
    SsdDramBackend *b = *mbe = g_malloc0(sizeof(SsdDramBackend));

    b->size = nbytes;
    b->logical_space = g_malloc0(nbytes);

    if (mlock(b->logical_space, nbytes) == -1)
    {
        femu_err("Failed to pin the memory backend to the host DRAM\n");
        g_free(b->logical_space);
        abort();
    }

    return 0;
}

void free_dram_backend(SsdDramBackend *b)
{
    if (b->logical_space)
    {
        munlock(b->logical_space, b->size);
        g_free(b->logical_space);
    }
}

/**
 *  执行底层的DMA数据读写
 *  @b:指向SSD后端存储的指针，包含了模拟内存分配的宿主机的DRAM空间
 *  @qsg:记录了数据在虚拟机内存中分散的物理地址片段
 *  @lbal:记录了数据在SSD后端内存中的逻辑地址
 *  @is_write:标识当前操作是读还是写
 */
int backend_rw(SsdDramBackend *b, QEMUSGList *qsg, uint64_t *lbal, bool is_write)
{
    // 当前正在处理sglist中的第几个内存片段
    int sg_cur_index = 0;

    // 当前所在的内存片段内部，已经处理了多少字节
    // 为什么要写？因为QEMU的底层dma_memory_rw 可能无法一次性拷完一个完整的片段
    dma_addr_t sg_cur_byte = 0;

    // cur_addr:正在操作的虚拟机物理地址gpa
    // cur_len：本次准备拷贝的字节长度
    dma_addr_t cur_addr, cur_len;

    // 数据在FEMU后端DRAM空间中的起始字节偏移量
    uint64_t mb_oft = lbal[0];

    // 指向FEMU为这个SSD申请的整块DRAM空间的起始虚拟地址（HVA）
    void *mb = b->logical_space;

    DMADirection dir = DMA_DIRECTION_FROM_DEVICE;

    if (is_write)
    {
        dir = DMA_DIRECTION_TO_DEVICE;
    }

    while (sg_cur_index < qsg->nsg)
    {
        // 正在操作的虚拟机物理地址 = 当前sg片段的起始地址 + 已经处理的字节数
        cur_addr = qsg->sg[sg_cur_index].base + sg_cur_byte;

        // 当前片段还剩下多少字节没有搬运
        cur_len = qsg->sg[sg_cur_index].len - sg_cur_byte;

        // QEMU提供的安全DMA访问接口：负责把虚拟机地址（cur_addr)和宿主机FEMU进程地址(mb+mb_oft)之间的数据搬运
        // 为什么使用这个接口而不是memcpy？因为HVA需要经过IOMMU映射
        if (dma_memory_rw(qsg->as, cur_addr, mb + mb_oft, cur_len, dir, MEMTXATTRS_UNSPECIFIED))
        {
            femu_err("dma_memory_rw error\n");
        }

        sg_cur_byte += cur_len;
        if (sg_cur_byte == qsg->sg[sg_cur_index].len)
        {
            sg_cur_byte = 0;
            ++sg_cur_index;
        }

        if (b->femu_mode == FEMU_OCSSD_MODE)
        {
            mb_oft = lbal[sg_cur_index];
        }
        else if (b->femu_mode == FEMU_BBSSD_MODE ||
                 b->femu_mode == FEMU_NOSSD_MODE ||
                 b->femu_mode == FEMU_ZNSSD_MODE)
        {
            mb_oft += cur_len;
        }
        else
        {
            assert(0);
        }
    }

    qemu_sglist_destroy(qsg);

    return 0;
}
