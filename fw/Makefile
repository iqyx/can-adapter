BINARY = main

OBJS += libuxb.o can.pb.o uxb_slave.o nanopb/pb_common.o nanopb/pb_encode.o nanopb/pb_decode.o can.o

CFLAGS += -Inanopb
LDSCRIPT = stm32f042g6u6.ld
LDFLAGS += --specs=rdimon.specs
LDFLAGS += --specs=nano.specs
LDLIBS	+= -lrdimon

include Makefile.include

proto:
	protoc --plugin=nanopb/generator/protoc-gen-nanopb --proto_path=. --nanopb_out=. --proto_path=nanopb/generator/proto *.proto
