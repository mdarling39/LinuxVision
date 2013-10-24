#ifndef NEON_INSTRUCTIONS_H_INCLUDED
#define NEON_INSTRUCTIONS_H_INCLUDED

#include <arm_neon.h>


/// Extracts the nth channel (assumes 3-ch input, 1-ch output)
void vector_extractch_of_n (uint8_t *ptr, uint8_t *ptr_out, uint32_t items, uint8_t ch)
{
    static uint8_t *i;
    static uint8_t *j;
    i=ptr;
    j=ptr_out;

    static uint8x16x3_t color8x16x3;
    static uint8x8x3_t color8x8x3;

    ///Operate 16 bytes at a time
    for (; i<=(ptr + items - 3*16); i+=3*16, j+=16)
    {
        color8x16x3 = vld3q_u8(i);   // load data into separate channels
        vst1q_u8(j, color8x16x3.val[ch]);         // store the desired channel
    }

    ///Operate 8 bytes at a time
    for (; i<=(ptr + items - 3*8); i+=3*8, j+=8)
    {
        color8x8x3 = vld3_u8(i);
        vst1_u8(j, color8x8x3.val[ch]);
    }

    ///Operate on single bytes
    for (; i<(ptr + items); i+=3, j++)
    {
        color8x8x3 = vld3_lane_u8(i, color8x8x3, 0);
        vst1_lane_u8(j, color8x8x3.val[ch], 0);
    }
}


/// Performs a binary threshold of a uint8_t array
void vector_binary_thresh_of_n (uint8_t *ptr, uint8_t *ptr_out, uint32_t items, uint8_t thresh)
{
    static uint8x16_t thresh_u = vdupq_n_u8(thresh); // assign threshold value

    static uint8_t *i;
    static uint8_t *j;
    i=ptr;
    j=ptr_out;

    static uint8x16_t v0_16, v1_16;
    static uint8x8_t v0_8;

    /// Operate 32 bytes at a time (2x16 bytes)
    for (; i <= (ptr + items - 32); i+=32, j+=32)
    {
        v0_16 = vld1q_u8(i);        // load the data
        v1_16 = vld1q_u8(i + 16);

        v0_16 = vcgtq_u8(v0_16, thresh_u);        // compare to threshold value
        v1_16 = vcgtq_u8(v1_16, thresh_u);

        vst1q_u8(j, v0_16);                    // store the data back to the original array
        vst1q_u8(j + 16, v1_16);

    }

    /// Operate 8 bytes at a time using 64-bit registers for remainder
    for (; i <= (ptr + items - 8); i+=8, j+= 8)
    {
        v0_8 = vld1_u8(i);
        v0_8 = vcgt_u8(v0_8, vget_low_u8(thresh_u));
        vst1_u8(j, v0_8);
    }

    /// Operate on remaining bites one-at a time
    for (; i < (ptr + items); i++, j++)
    {
        v0_8 = vld1_lane_u8(i, v0_8, 0);
        v0_8 = vcgt_u8(v0_8, vget_low_u8(thresh_u));
        vst1_lane_u8(j, v0_8, 0);
    }

}


/// Performs a dilation operation on an image using a kernel matrix
void vector_dilate (uint8_t *ptr, uint8_t *ptr_out, const int nl, const int nc, uint8_t *ptr_knl, int knl_sz)
{

// going to overlay the image and do a bitwise "OR". Will simulate memcopies by
// moving the pointers to the copied data. Will use the mask to know how many
// loops are required.

    const unsigned int items = nl*nc;
    const uint8_t n_knl = knl_sz / 2;

    // copy the image to the outimage
    if (ptr != ptr_out)
        memcpy(ptr_out,ptr,items);

    // allocate space for a temporary image + some extra zero padding
    const size_t padding = n_knl*nc + n_knl; // padding for one side
    const size_t sz = items + 2*padding;
    uint8_t *buff;
    buff = (uint8_t*) malloc(sz);
    uint8_t *b = buff + padding;       // return a pointer to the begining of the memory block
    memset(buff,0,padding);       // we are interested in, copy the input image, and
    memcpy(b,ptr,items);            // pad with zeros.
    memset(b+items,0,padding);


    for (int k=0; k<(knl_sz*knl_sz); k++)
    {
        if (ptr_knl[k] == 0)  // if the kernel has a false value, move on
            continue;

        int rowShift = (int)(k/knl_sz) - n_knl;
        int colShift = (int)(k%knl_sz) - n_knl;
        uint8_t *shift;
        shift = b + rowShift*nc + colShift;

        // C implementation:
        //
        //for (int kk=0; kk<items; kk++)
        //{
        //  ptr_out[kk] |= shift[kk];
        //}
        //

        // NEON implementation
        // Do 32 bytes at a time using two registers of 16-bytes each
        int kk=0;
        for (; kk < (items - 32); kk+=32)
        {
            uint8x16_t v0 = vld1q_u8(ptr_out + kk);
            uint8x16_t v1 = vld1q_u8(ptr_out + kk + 16);
            uint8x16_t w0 = vld1q_u8(shift + kk);
            uint8x16_t w1 = vld1q_u8(shift + kk + 16);

            // Apply the logical OR
            v0 = vorrq_u8(v0,w0);
            v1 = vorrq_u8(v1,w1);

            // store back to memory
            vst1q_u8(ptr_out + kk,v0);
            vst1q_u8(ptr_out + kk + 16,v1);
        }

        // If any chunks of 8-bytes, do those
        for (; kk < (items - 8); kk+= 8)
        {
            uint8x8_t v0 = vld1_u8(ptr_out + kk);
            uint8x8_t w0 = vld1_u8(shift + kk);
            v0 = vorr_u8(v0,w0);
            vst1_u8(ptr_out + kk,v0);
        }

        // Then do any remaining values by loading single lanes
        for (; kk < items; kk++)
        {
            uint8x8_t v0,w0;
            v0 = vld1_lane_u8(ptr_out + kk,v0,0);
            w0 = vld1_lane_u8(shift + kk,w0,0);
            v0 = vorr_u8(v0,w0);
            vst1_lane_u8(ptr_out + kk,v0,0);
        }
        // END NEON implementation
    }


    free(buff);
    b = NULL;
    buff = NULL;
}

/*
/// Performs a dilation operation on an image of known dimensions for a 5x5 kernel matrix
void vector_dilate_5x5 (uint8_t *ptr, uint8_t *ptr_out, const uint8_t *ptr_knl, uint16_t nl, uint16_t nc, uint16_t items)
{
    // 5x5 kernel
    #define NM1 3
    #define N   4
    #define NP1 5
    #define NO2 2

    uint8_t *i = ptr;
    uint8_t *j = ptr_out;
    const uint8_t *k = ptr_knl;

    static uint8x16_t kernel = vld1q_u8(k);                                // load the kernel vectors

    for (i=ptr+(NO2*nc), j+=(NO2*nc); i<(ptr + nc*(nl - NO2)); i++, j++)
    {
        // load the matrix elements
        // (I hope each of these loads doesn't cost an cpu cycle)
        static uint8x16_t v0;

        v0 = vld1q_lane_u8(i - 2 - 2*nc, v0, 0); // first row
        v0 = vld1q_lane_u8(i - 1 - 2*nc, v0, 1);
        v0 = vld1q_lane_u8(i     - 2*nc, v0, 2);
        v0 = vld1q_lane_u8(i + 1 - 2*nc, v0, 3);
        v0 = vld1q_lane_u8(i + 2 - 2*nc, v0, 4);

        v0 = vld1q_lane_u8(i - 2 - nc, v0, 5); // second row
        v0 = vld1q_lane_u8(i - 1 - nc, v0, 6);
        v0 = vld1q_lane_u8(i     - nc, v0, 7);
        v0 = vld1q_lane_u8(i + 1 - nc, v0, 8);
        v0 = vld1q_lane_u8(i + 2 - nc, v0, 9);

        v0 = vld1q_lane_u8(i - 2, v0, 10); // middle row
        v0 = vld1q_lane_u8(i - 1, v0, 11);
        v0 = vld1q_lane_u8(i    , v0, 12);
        v0 = vld1q_lane_u8(i + 1, v0, 13);
        v0 = vld1q_lane_u8(i + 2, v0, 14);

        v0 = vld1q_lane_u8(i - 2 + nc, v0, 15); // fourth row

        // Perform an AND operation on the bytes
        v0 = vandq_u8(kernel, v0);
        static uint8x8_t result = vorr_u8(vget_low_u8(v0),vget_high_u8(v0));

        // check to see if any bytes matched up and store value accordingly
        static uint64x1_t result64 = vreinterpret_u64_u8(result);
        if (result64 > 0)
        {
            *j = 0xFF;
        } else {

        // load in what we haven't checked yet
        v0 = vld1q_lane_u8(i - 1 + nc, v0, 0); // rest of fourth row
        v0 = vld1q_lane_u8(i     + nc, v0, 1);
        v0 = vld1q_lane_u8(i + 1 + nc, v0, 2);
        v0 = vld1q_lane_u8(i + 2 + nc, v0, 3);

        v0 = vld1q_lane_u8(i - 2 + 2*nc, v0, 4); // fifth row
        v0 = vld1q_lane_u8(i - 1 + 2*nc, v0, 5);
        v0 = vld1q_lane_u8(i     + 2*nc, v0, 6);
        v0 = vld1q_lane_u8(i + 1 + 2*nc, v0, 7);
        v0 = vld1q_lane_u8(i + 2 + 2*nc, v0, 8);

        v0 = vandq_u8(kernel, v0);
        result = vorr_u8(vget_low_u8(v0),vget_high_u8(v0));
        result64 = vreinterpret_u64_u8(result);
        if (result64 > 0)
            *j = 0xFF;
        else
            *j = 0x00;

        }

        // Border handling
        if ( ((i-ptr) % nc) == (nc - NO2))
        {
            i += NM1;  // additional shift will be generated in for loop
            j += NM1;
        }
    }
}
*/

#endif // NEON_INSTRUCTIONS_H_INCLUDED
