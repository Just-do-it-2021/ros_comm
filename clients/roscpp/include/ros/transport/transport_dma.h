#ifndef ROSCPP_TRANSPORT_DMA_H
#define ROSCPP_TRANSPORT_DMA_H

#include <string>

#include <ros/transport/transport.h>

#include <boost/thread/mutex.hpp>
#include <boost/smart_ptr.hpp>
#include "ros/io.h"

namespace ros {
  class TransportDMA;
  typedef boost::shared_ptr<TransportDMA> TransportDMAPtr;

  class PollSet;

  class ROSCPP_DECL TransportDMA: public Transport {
    public:
      enum Flags {
        SYNCHRONOUS = 1<<0,
      };

      TransportDMA(PollSet* poll_set, int flags = 0);
      virtual ~TransportDMA();

      //create client
      TransportDMAPtr CreateOutgoing();

      //create server
      bool CreateIncoming(bool is_server);

      virtual int32_t read(uint8_t* buffer, uint32_t size) override;

      virtual int32_t write(uint8_t* buffer, uint32_t size) override;

      virtual void enableWrite() override;

      virtual void disableWrite() override;

      virtual void enableRead() override;

      virtual void disableRead() override;

      virtual void close() override;

      virtual const char* getType() override { return "DMAROS"; };

      virtual std::string getTransportInfo() override;

      virtual bool requiresHeader() override { return false; }

    private:
      bool initializeDMAHandle();

      void DMAHandleUpdate(int events);

      bool is_server_;

      int dma_handle_;

      PollSet* poll_set_;

      int flags_;

      boost::mutex close_mutex_;

      bool expecting_read_;
      bool expecting_write_;

      bool closed_;
  };
}

#endif