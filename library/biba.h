#ifndef __H_BIBA_LIB
#define __H_BIBA_LIB

#include <stdint.h>
#include <byteswap.h>
#include <stddef.h>
#include <assert.h>
#include <stdio.h>
#include "scoba/dynarr.h"
#include <endian.h>

// Change the endianess of __count__ elements with size __length__
// in contiguous memory gained by __ptr__
void change_byte_endianess(void* ptr, size_t count, size_t length) {
    if(length <= 1) { 
        assert(0 || "Length so small...");
        return;
    }

    size_t index = 0;
    switch(length){
        case 2: {
            uint16_t* pointer = (uint16_t*)ptr;
            for(; index < count; index++) {
                pointer[index] = bswap_16(pointer[index]);
            }
            return;
        } break;
        case 4: {
            uint32_t* pointer = (uint32_t*)ptr;
            for(; index < count; index++) {
                pointer[index] = bswap_32(pointer[index]);
            }
            return;
        } break;
        case 8: {
            uint64_t* pointer = (uint64_t*)ptr;
            for(; index < count; index++) {
                pointer[index] = bswap_64(pointer[index]);
            }
            return;
        } break;
    }

    uint8_t* pointer = ptr;
    size_t lengthHalf = length/2;
    for(; index < count; index++) {
        uint8_t buffer;
        size_t jndex = 0;
        for(; jndex < lengthHalf; jndex++){
            buffer = pointer[jndex];
            pointer[jndex] = pointer[length - jndex - 1];
            pointer[length - jndex - 1] = buffer;
        }
        pointer += length;
    }
}

void swap_host_local_endian(void* ptr, size_t count, size_t length) {
    if(*(uint8_t *)(uint32_t[]){1}) {
        change_byte_endianess(ptr, count, length);
    }
}

struct biba_pdu {
    uint8_t fun_code;
    uint8_t* data;
};

typedef struct biba_pdu mb_req_pdu;
typedef struct biba_pdu mb_rsp_pdu;
typedef struct biba_pdu mb_excep_rsp_pdu;

// limited to 256 bytes with RS232/RS485 and 260 bytes with TCP
struct biba_adu {
    struct biba_pdu pdu;
};

typedef uint16_t in_reg; // RO
typedef uint16_t hold_reg; // RW
// disc_in - 1 bit RW
// coils - 1 bit RO

#define RD_DISCR_INPUTS (uint8_t)0x02
#define RD_COILS (uint8_t)0x01
#define WR_SING_COIL (uint8_t)0x05
#define WR_MULT_COILS (uint8_t)0x0F
#define RD_INPT_REG (uint8_t)0x04
#define RD_HOLD_REGS (uint8_t)0x03
#define WR_SING_REG (uint8_t)0x06
#define WR_MULT_REGS (uint8_t)0x10
#define RDWR_MULT_REGS (uint8_t)0x17
    // some other stuff

typedef struct {
    FILE* channel;
} biba_channel;

typedef struct {
    uint8_t* elements;
    size_t count;
    size_t capacity;
} uint8_dynarr;

#define BIBA_ERROR_RESPONSE 0

typedef struct {
    uint8_t* elements;
    uint8_t count;
} biba_rd_coils_ok_resp;

typedef struct {
    uint8_t type;
    union {
        uint8_t error;
        biba_rd_coils_ok_resp resp;
    } items;
} biba_rd_coils_resp;

biba_rd_coils_resp read_coils(biba_channel* biba, uint16_t start_addr, uint16_t count) {
    if(count > 2000) {
        LOGWARN(stderr, "Count(%d) is bigger than allowed(2000)", count);
        return (biba_rd_coils_resp){0};
    }
    size_t tx_count = 0;

    uint8_dynarr buffer = {};
    DYNARR_INIT(&buffer);
    //swap bytes if host is little-endian
    start_addr = htobe16(start_addr);
    count = htobe16(count);
    DYNARR_APPEND(&buffer, RD_COILS);
    DYNARR_APPEND(&buffer, ((uint8_t*)(&start_addr))[0]);
    DYNARR_APPEND(&buffer, ((uint8_t*)(&start_addr))[1]);
    DYNARR_APPEND(&buffer, ((uint8_t*)(&count))[0]);
    DYNARR_APPEND(&buffer, ((uint8_t*)(&count))[1]);

    tx_count = fwrite(buffer.elements, 1, buffer.count, biba->channel);
    DYNARR_FREE(&buffer);
    if(tx_count != buffer.count) {
        LOGWARN(stderr, "something is occured when writing to the stream. buffer: %d -> transmitted: %d", buffer.count, tx_count);
        return (biba_rd_coils_resp){0};
    }

    // reading response code and (N or exception code)
    uint8_t response_code[2];
    tx_count = fread(response_code, 1, 2, biba->channel);
    if(tx_count != 2) {
        LOGWARN(stderr, "something is occured when reading from the stream. buffer: %d -> received: %d", 2, tx_count);
        return (biba_rd_coils_resp){0};
    }
    biba_rd_coils_resp response = {0};
    switch(response_code[0]){
        case RD_COILS: {
            response.type = 1;
            response.items.resp.count = response_code[1];
            response.items.resp.elements = malloc(response_code[1] * sizeof(&(response.items.resp.elements)));
            
            if(response.items.resp.elements == NULL) {
                LOGERROR(stderr, errno, "Can't allocate memory for response buffer");
                return (biba_rd_coils_resp){0};
            }

            tx_count = fread(response.items.resp.elements, 1, response_code[1], biba->channel);
            if(tx_count != response_code[1]) {
                LOGWARN(stderr, "Something is occured when reading from the stream. buffer: %d -> received: %d", response_code[1], tx_count);
                free(response.items.resp.elements);
                return (biba_rd_coils_resp){0};
            }

            return response;
        } break;
        case RD_COILS + 0x80: {
            response.type = BIBA_ERROR_RESPONSE;
            response.items.error = response_code[1];
            return response;
        } break;
    }
    assert(0 && "unreachable");
    return (biba_rd_coils_resp){0};
}

#endif
