#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <future>
#include <list>
#include <chrono>
// driver/common
#include <vortex.h>
#include <vx_malloc.h>
#include <vx_utils.h>

#include <VX_config.h>
// sim/common
#include <mem.h>
#include <util.h>
#include <processor.h>

#define RAM_PAGE_SIZE 4096

using namespace vortex; 

///////////////////////////////////////////////////////////////////////////////

class vx_device;
/**
 * @brief 用于在设备和主机之间交换内存的缓冲
 */
class vx_buffer {
public:

    vx_buffer(uint64_t size, vx_device* device) 
        : size_(size)
        , device_(device) {
        auto aligned_asize = aligned_size(size, CACHE_BLOCK_SIZE); //返回一个CACHE_BLOCK_SIZE的大小
        data_ = malloc(aligned_asize);
    }

    ~vx_buffer() {
        if (data_) {
            free(data_);
        }
    }
    /// @brief 返回data成员
    /// @return data_
    void* data() const {
        return data_;
    }
    /// @brief 返回size_成员
    /// @return size_
    uint64_t size() const {
        return size_;
    }
    /// @brief 返回device_成员
    /// @return device_
    vx_device* device() const {
        return device_;
    }

private:
    uint64_t size_; ///< buffer大小
    vx_device* device_; ///< 设备
    void* data_;///< 数据
};

///////////////////////////////////////////////////////////////////////////////

class vx_device {    
public:
    vx_device() 
        : ram_(RAM_PAGE_SIZE)
        , mem_allocator_(
            ALLOC_BASE_ADDR,
            ALLOC_BASE_ADDR + LOCAL_MEM_SIZE,
            RAM_PAGE_SIZE,
            CACHE_BLOCK_SIZE) 
    {
        processor_.attach_ram(&ram_); //表示GPGPU,不包含显存/主存?
    }

    ~vx_device() {    
        if (future_.valid()) {
            future_.wait();
        }
    }

    int alloc_local_mem(uint64_t size, uint64_t* dev_maddr) {
        return mem_allocator_.allocate(size, dev_maddr);
    }

    int free_local_mem(uint64_t dev_maddr) {
        return mem_allocator_.release(dev_maddr);
    }
    /**
    * @brief 往设备内存(成员变量ram_)中写入数据
    * @param [in] src: 为要写的数据的起始地址，
    * @param [in] dest_addr: 要写的地址
    * @param [in] size: 要写入的大小
    * @param [in] src_offset: 起始地址偏移
    */
    int upload(const void* src, uint64_t dest_addr, uint64_t size, uint64_t src_offset) {
        uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
        if (dest_addr + asize > LOCAL_MEM_SIZE)
            return -1;

        /*printf("VXDRV: upload %ld bytes from 0x%lx:", size, uintptr_t((uint8_t*)src + src_offset));
        for (int i = 0;  i < (asize / CACHE_BLOCK_SIZE); ++i) {
            printf("\n0x%08lx=", dest_addr + i * CACHE_BLOCK_SIZE);
            for (int j = 0;  j < CACHE_BLOCK_SIZE; ++j) {
                printf("%02x", *((uint8_t*)src + src_offset + i * CACHE_BLOCK_SIZE + CACHE_BLOCK_SIZE - 1 - j));
            }
        }
        printf("\n");*/
        
        ram_.write((const uint8_t*)src + src_offset, dest_addr, asize);
        return 0;
    }
    
    /**
     * @brief 将设备内存(成员变量ram_)中的数据读出
     * @param  dest             存放要读取的数据的起始地址
     * @param  src_addr         要读取的内存地址
     * @param  size             要读取的大小
     * @param  dest_offset      起始地址偏移
     * @return int 
     */
    int download(void* dest, uint64_t src_addr, uint64_t size, uint64_t dest_offset) {
        uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
        if (src_addr + asize > LOCAL_MEM_SIZE)
            return -1;

        ram_.read((uint8_t*)dest + dest_offset, src_addr, asize);
        
        /*printf("VXDRV: download %ld bytes to 0x%lx:", size, uintptr_t((uint8_t*)dest + dest_offset));
        for (int i = 0;  i < (asize / CACHE_BLOCK_SIZE); ++i) {
            printf("\n0x%08lx=", src_addr + i * CACHE_BLOCK_SIZE);
            for (int j = 0;  j < CACHE_BLOCK_SIZE; ++j) {
                printf("%02x", *((uint8_t*)dest + dest_offset + i * CACHE_BLOCK_SIZE + CACHE_BLOCK_SIZE - 1 - j));
            }
        }
        printf("\n");*/
        
        return 0;
    }
    /**
     * @brief 设备开始执行
     * 多线程执行
     * 
     * @return int 
     */
    int start() {   
        // ensure prior run completed
        if (future_.valid()) { //如果还没有运行结束
            future_.wait();
        }
        // start new run
        future_ = std::async(std::launch::async, [&]{
            processor_.run();
        });
        return 0;
    }
/**
 * @brief 等待设备执行，以毫秒计
 * @param  timeout  等待时间
 * @return int 
 */
    int wait(uint64_t timeout) {
        if (!future_.valid())
            return 0;
        uint64_t timeout_sec = timeout / 1000;
        std::chrono::seconds wait_time(1);
        for (;;) {
            // wait for 1 sec and check status
            auto status = future_.wait_for(wait_time);
            if (status == std::future_status::ready 
             || 0 == timeout_sec--)
                break;
        }
        return 0;
    }

private:


    RAM ram_; /**< 设备内存， 定义在/sim/common/mem.h */
    Processor processor_;/**< 设备*/
    MemoryAllocator mem_allocator_; /**< 页表管理 */
    std::future<void> future_;/**< 用于processor与其他代码同时运行时，检测processor的执行情况 */
};

///////////////////////////////////////////////////////////////////////////////

#ifdef DUMP_PERF_STATS

class AutoPerfDump {
private:
    std::list<vx_device_h> devices_;

public:
    AutoPerfDump() {} 

    ~AutoPerfDump() {
        for (auto device : devices_) {
            vx_dump_perf(device, stdout);
        }
    }

    void add_device(vx_device_h device) {
        devices_.push_back(device);
    }

    void remove_device(vx_device_h device) {
        devices_.remove(device);
    }    
};

AutoPerfDump gAutoPerfDump;
#endif

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief 更改设备配置
 * @param  hdevice          设备
 * @param  caps_id          设置项目
 * @param  value            设置值
 * @return int 
 */
extern int vx_dev_caps(vx_device_h hdevice, uint32_t caps_id, uint64_t *value) {
   if (nullptr == hdevice)
        return  -1;

    switch (caps_id) {
    case VX_CAPS_VERSION:
        *value = IMPLEMENTATION_ID;
        break;
    case VX_CAPS_MAX_CORES:
        *value = NUM_CORES * NUM_CLUSTERS;        
        break;
    case VX_CAPS_MAX_WARPS:
        *value = NUM_WARPS;
        break;
    case VX_CAPS_MAX_THREADS:
        *value = NUM_THREADS;
        break;
    case VX_CAPS_CACHE_LINE_SIZE:
        *value = CACHE_BLOCK_SIZE;
        break;
    case VX_CAPS_LOCAL_MEM_SIZE:
        *value = LOCAL_MEM_SIZE;
        break;
    case VX_CAPS_ALLOC_BASE_ADDR:
        *value = ALLOC_BASE_ADDR;
        break;
    case VX_CAPS_KERNEL_BASE_ADDR:
        *value = STARTUP_ADDR;
        break;
    default:
        std::cout << "invalid caps id: " << caps_id << std::endl;
        std::abort();
        return -1;
    }

    return 0;
}
/**
 * @brief 打开一个新设备，使传入的设备指针指向新的对象
 * @param  hdevice          My Param doc
 * @return int 
 */
extern int vx_dev_open(vx_device_h* hdevice) {
    if (nullptr == hdevice)
        return  -1;

    *hdevice = new vx_device();

#ifdef DUMP_PERF_STATS
    gAutoPerfDump.add_device(*hdevice);
#endif

    return 0;
}
/**
 * @brief 执行结束释放设备
 * @param  hdevice          My Param doc
 * @return int 
 */
extern int vx_dev_close(vx_device_h hdevice) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);
    
#ifdef DUMP_PERF_STATS
    gAutoPerfDump.remove_device(hdevice);
    vx_dump_perf(hdevice, stdout);
#endif

    delete device;

    return 0;
}
/// 为device分配内存空间
extern int vx_mem_alloc(vx_device_h hdevice, uint64_t size, uint64_t* dev_maddr) {
    if (nullptr == hdevice 
     || nullptr == dev_maddr
     || 0 >= size)
        return -1;

    vx_device *device = ((vx_device*)hdevice);
    return device->alloc_local_mem(size, dev_maddr);
}
/// 释放device内存空间
extern int vx_mem_free(vx_device_h hdevice, uint64_t dev_maddr) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);
    return device->free_local_mem(dev_maddr);
}
/// hbuffer指向vx_buffer
extern int vx_buf_alloc(vx_device_h hdevice, uint64_t size, vx_buffer_h* hbuffer) {
    if (nullptr == hdevice 
     || 0 >= size
     || nullptr == hbuffer)
        return -1;

    vx_device *device = ((vx_device*)hdevice);

    auto buffer = new vx_buffer(size, device);
    if (nullptr == buffer->data()) {
        delete buffer;
        return -1;
    }

    *hbuffer = buffer;

    return 0;
}
//vx_buffer->data是什么？
extern void* vx_host_ptr(vx_buffer_h hbuffer) {
    if (nullptr == hbuffer)
        return nullptr;

    vx_buffer* buffer = ((vx_buffer*)hbuffer);

    return buffer->data();
}

extern int vx_buf_free(vx_buffer_h hbuffer) {
    if (nullptr == hbuffer)
        return -1;

    vx_buffer* buffer = ((vx_buffer*)hbuffer);

    delete buffer;

    return 0;
}
/**
 * @brief 将数据拷贝到设备内存空间(成员变量ram_.write())
 * @note vx_buffer是device及其memory的缓冲区，buffer->data为memory中的数据
 * @code 
 * auto buffer = (vx_buffer*)hbuffer;
 * @endcode
 * 
 * @param  hbuffer      缓冲区指针
 * @param  dev_maddr    要写入设备内存地址指针
 * @param  size         要写入的大小
 * @param  src_offset   写入地址在块内的偏移
 * @return int 
 */
extern int vx_copy_to_dev(vx_buffer_h hbuffer, uint64_t dev_maddr, uint64_t size, uint64_t src_offset) {
    if (nullptr == hbuffer 
     || 0 >= size)
        return -1;

    auto buffer = (vx_buffer*)hbuffer;

    if (size + src_offset > buffer->size())
        return -1;

    return buffer->device()->upload(buffer->data(), dev_maddr, size, src_offset);
}

extern int vx_copy_from_dev(vx_buffer_h hbuffer, uint64_t dev_maddr, uint64_t size, uint64_t dest_offset) {
     if (nullptr == hbuffer 
      || 0 >= size)
        return -1;

    auto buffer = (vx_buffer*)hbuffer;

    if (size + dest_offset > buffer->size())
        return -1;    

    return buffer->device()->download(buffer->data(), dev_maddr, size, dest_offset);
}

extern int vx_start(vx_device_h hdevice) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);

    return device->start();
}

extern int vx_ready_wait(vx_device_h hdevice, uint64_t timeout) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);

    return device->wait(timeout);
}