#include <stdio.h>
#include <string>

#ifndef __X3DS_IO_INTERFACE
#define __X3DS_IO_INTERFACE

namespace x3ds {
    class IOInterface {

        public:
            explicit IOInterface(std::string base_directory);
            virtual void write(std::string filename, std::string data);
            virtual void write_chunk(std::string filename, std::string data, uint32_t chunk_number);

            virtual std::string read(std::string filename);
            virtual std::string read_chunk(std::string filename, uint32_t chunk_number);

            virtual void join_chunks(std::string filename);
    };
}

#endif