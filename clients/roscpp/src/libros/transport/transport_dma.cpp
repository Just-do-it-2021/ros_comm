#include <sstream>
#include <boost/bind/placeholders.hpp>
#include <boost/bind.hpp>

#include "ros/transport/transport_dma.h"
#include "ros/poll_set.h"
#include "ros/file_log.h"
#include "ros/io.h"

namespace ros {
  TransportDMA::TransportDMA(PollSet* poll_set, int flags)
    : is_server_(false)
    , dma_handle_(-1)
    , poll_set_(poll_set)
    , flags_(flags)
    , expecting_read_(false)
    , expecting_write_(false)
    , closed_(false) {
  }


  TransportDMA::~TransportDMA(){
    // TODO: 注销dma通道
  }

  TransportDMAPtr TransportDMA::CreateOutgoing() {
    ROS_ASSERT(is_server_);

    TransportDMAPtr transport(boost::make_shared<TransportDMA>(poll_set_, flags_));
    if (transport->initializeDMAHandle()) {
      ROS_ERROR("DMA Failed to create outgoing connection");
      return TransportDMAPtr();
    }

    return transport;
  }

  bool TransportDMA::CreateIncoming(bool is_server) {
  #if 0
    is_server_ = is_server;

    if(!initializeDMAHandle()) {
      return false;
    }

    enableRead();
  #else
    (void)is_server;
  #endif

    return true;
  }

  int32_t TransportDMA::read(uint8_t* buffer, uint32_t size) {
    (void)buffer;
    (void)size;
    return 0;
  }

  int32_t TransportDMA::write(uint8_t* buffer, uint32_t size) {
    (void)buffer;
    (void)size;
    return 0;
  }

  void TransportDMA::enableWrite() {
    {
      boost::mutex::scoped_lock lock(close_mutex_);

      if (closed_) {
        return;
      }
    }

    if (!expecting_write_) {
      poll_set_->addEvents(dma_handle_, POLLOUT);
      expecting_write_ = true;
    }
  }

  void TransportDMA::disableWrite() {
    {
      boost::mutex::scoped_lock lock(close_mutex_);

      if (closed_) {
        return;
      }
    }

    if (expecting_write_) {
      poll_set_->delEvents(dma_handle_, POLLOUT);
    }
  }

  void TransportDMA::enableRead() {
    {
      boost::mutex::scoped_lock lock(close_mutex_);

      if (closed_) {
        return;
      }
    }

    if (!expecting_read_) {
      poll_set_->addEvents(dma_handle_, POLLIN);
      expecting_read_ = true;
    }
  }

  void TransportDMA::disableRead() {
    {
      boost::mutex::scoped_lock lock(close_mutex_);

      if (closed_) {
        return;
      }
    }

    if (expecting_read_) {
      poll_set_->delEvents(expecting_read_, POLLIN);
    }
  }

  void TransportDMA::close() {
    Callback disconnect_cb;

    if (!closed_)
    {
      {
        boost::mutex::scoped_lock lock(close_mutex_);

        if (!closed_)
        {
          closed_ = true;

          ROSCPP_LOG_DEBUG("DMA handle [%d] closed", dma_handle_);

          //TODO:
          //ROS_ASSERT(dma_handle_ != ROS_INVALID_SOCKET);

          if (poll_set_)
          {
            poll_set_->delSocket(dma_handle_);
          }

          //TODO: 注销dma通道

          dma_handle_ = ROS_INVALID_SOCKET;

          disconnect_cb = disconnect_cb_;

          disconnect_cb_ = Callback();
          read_cb_ = Callback();
          write_cb_ = Callback();
        }
      }
    }

    if (disconnect_cb)
    {
      disconnect_cb(shared_from_this());
    }
  }

  std::string TransportDMA::getTransportInfo() {
    //TODO
    return std::string();
  }

  bool TransportDMA::initializeDMAHandle() {
    //TODO: 申请pcie句柄

    //TODO： 申请dma通道

    //TODO: 将dma物理地址映射到内存

    //TODO： 注册DMA接收回调函数

    //ROS_ASSERT(dma_handle_ != ROS_INVALID_SOCKET);

    if (!(flags_ & SYNCHRONOUS)) {
      int result = set_non_blocking(dma_handle_);
      if ( result != 0 ) {
        ROS_ERROR("setting dma handle [%d] as non_blocking failed with error [%d]", dma_handle_, result);
        close();
        return false;
      }
    }

    ROS_ASSERT(poll_set_ || (flags_ & SYNCHRONOUS));
    if (poll_set_) {
      poll_set_->addSocket(dma_handle_, boost::bind(&TransportDMA::DMAHandleUpdate, this, boost::placeholders::_1), shared_from_this());
    }
    return true;
  }

  void TransportDMA::DMAHandleUpdate(int events) {
    {
      boost::mutex::scoped_lock lock(close_mutex_);

      if (closed_)
      {
        return;
      }
    }

    if((events & POLLERR) ||
      (events & POLLHUP) ||
      (events & POLLNVAL))
    {
      ROSCPP_LOG_DEBUG("DMA handle %d closed with (ERR|HUP|NVAL) events %d", dma_handle_, events);
      close();
    }
    else
    {
      if ((events & POLLIN) && expecting_read_)
      {
        if (read_cb_)
        {
          read_cb_(shared_from_this());
        }
      }

      if ((events & POLLOUT) && expecting_write_)
      {
        if (write_cb_)
        {
          write_cb_(shared_from_this());
        }
      }
    }
  }
}