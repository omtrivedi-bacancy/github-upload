/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS7000 heartrate application
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file fft.c
 *
 *  \brief FFT-related functions
 *
 */

#include "fft.h"
#include <math.h>



/*
 *****************************************************************************
 * DEFINES  
 *****************************************************************************
 */


/*
 *****************************************************************************
 * VARIABLES
 *****************************************************************************
 */

// Pseudo code for generation of the Twiddle Coefficients...
// for (i=0; i<(3*FFT_N/4); i++)
// {
//     twiddleCoef_r[i] = twiddleCoef[2*i]   = cos(i * 2 * PI / (double)FFT_N);
//     twiddleCoef_i[i] = twiddleCoef[2*i+1] = sin(i * 2 * PI / (double)FFT_N);
// }
// Note that cos(i * 2 * PI / (double)FFT_N) = sin((i+FFT_N/4) * 2 * PI / (double)FFT_N)
//   therefore twiddleCoef_r_int16_256[i] = twiddleCoef_i_int16_256[i+64]
// Note twiddleCoef_i_int16_256[256 - rIndex + 64] = twiddleCoef_i_int16_256[rIndex + 64]

// Twiddle Coefficients for 16-bit 256-bin FFT
static const int16_t twiddleCoef_i_int16_256[193] = {
         0,     804,    1608,    2411,    3212,    4011,    4808,    5602,
      6393,    7180,    7962,    8740,    9512,   10279,   11039,   11793,
     12540,   13279,   14010,   14733,   15447,   16151,   16846,   17531,
     18205,   18868,   19520,   20160,   20788,   21403,   22006,   22595,
     23170,   23732,   24279,   24812,   25330,   25833,   26320,   26791,
     27246,   27684,   28106,   28511,   28899,   29269,   29622,   29957,
     30274,   30572,   30853,   31114,   31357,   31581,   31786,   31972,
     32138,   32286,   32413,   32522,   32610,   32679,   32729,   32758,
     32767,   32758,   32729,   32679,   32610,   32522,   32413,   32286,
     32138,   31972,   31786,   31581,   31357,   31114,   30853,   30572,
     30274,   29957,   29622,   29269,   28899,   28511,   28106,   27684,
     27246,   26791,   26320,   25833,   25330,   24812,   24279,   23732,
     23170,   22595,   22006,   21403,   20788,   20160,   19520,   18868,
     18205,   17531,   16846,   16151,   15447,   14733,   14010,   13279,
     12540,   11793,   11039,   10279,    9512,    8740,    7962,    7180,
      6393,    5602,    4808,    4011,    3212,    2411,    1608,     804,
         0,    -804,   -1608,   -2411,   -3212,   -4011,   -4808,   -5602,
     -6393,   -7180,   -7962,   -8740,   -9512,  -10279,  -11039,  -11793,
    -12540,  -13279,  -14010,  -14733,  -15447,  -16151,  -16846,  -17531,
    -18205,  -18868,  -19520,  -20160,  -20788,  -21403,  -22006,  -22595,
    -23170,  -23732,  -24279,  -24812,  -25330,  -25833,  -26320,  -26791,
    -27246,  -27684,  -28106,  -28511,  -28899,  -29269,  -29622,  -29957,
    -30274,  -30572,  -30853,  -31114,  -31357,  -31581,  -31786,  -31972,
    -32138,  -32286,  -32413,  -32522,  -32610,  -32679,  -32729,  -32758,
    -32768,
};

// Pseudo code for generation of the Twiddle A and B coefficients...
// for (i=0; i<FFT_N/2; i++)
// {
//     twiddleCoef_Ar[i]    = 0.5 * (1.0 - sin (2 * PI / (double)(2 * FFT_N) * (double)i));
//     twiddleCoef_A[2*i]   = 0.5 * (1.0 - sin (2 * PI / (double)(2 * FFT_N) * (double)i));
//     twiddleCoef_A[2*i+1] = 0.5 * (-1.0 * cos (2 * PI / (double)(2 * FFT_N) * (double)i));
//     twiddleCoef_B[2*i]   = 0.5 * (1.0 + sin (2 * PI / (double)(2 * FFT_N) * (double)i));
//     twiddleCoef_B[2*i+1] = 0.5 * (1.0 * cos (2 * PI / (double)(2 * FFT_N) * (double)i));
// }

// Twiddle A and B coefficients for 16-bit 256-bin FFT (used for 512-bin result)
// Note that Br=(1.0-Ar)=(0x8000-Ar), Bi=(-Ai)
// Note that Ar[(N/2)-i]=Ar[i], Ai[(N/2)-i]=-Ai[i]
// Note that Ai[i]=Ar[(N/4)-i]-0x4000
static const int16_t twiddleCoef_Ar_int16_256[128] = {
     16384,   16183,   15982,   15781,   15580,   15379,   15179,   14978,
     14778,   14578,   14378,   14179,   13980,   13781,   13583,   13385,
     13188,   12991,   12794,   12598,   12403,   12208,   12014,   11821,
     11628,   11436,   11245,   11054,   10864,   10676,   10487,   10300,
     10114,    9929,    9745,    9561,    9379,    9198,    9018,    8839,
      8661,    8484,    8308,    8134,    7961,    7789,    7619,    7449,
      7282,    7115,    6950,    6786,    6624,    6463,    6304,    6146,
      5990,    5835,    5682,    5531,    5381,    5233,    5087,    4942,
      4799,    4657,    4518,    4380,    4244,    4110,    3978,    3847,
      3719,    3592,    3468,    3345,    3224,    3105,    2989,    2874,
      2761,    2651,    2542,    2435,    2331,    2229,    2128,    2030,
      1935,    1841,    1749,    1660,    1573,    1488,    1406,    1325,
      1247,    1171,    1098,    1027,     958,     891,     827,     765,
       705,     648,     593,     541,     491,     443,     398,     355,
       315,     277,     241,     208,     177,     149,     123,     100,
        79,      60,      44,      31,      20,      11,       5,       1,
};

// Pseudo code for generation of the bit-reversal-table...
// for (k=1; k<=FFT_N/4; k++)
// {
//     for (i=0; i<log2(FFT_N); i++)
//     {
//         a[i] = k & (1 << i);
//     }
//     for (j=0; j<log2(FFT_N); j++)
//     {
//         if (a[j] != 0)
//             y[k] += (1 << ((log2(FFT_N)-1)-j));
//     }
//     y[k] = y[k] >> 1;
// }
// FFT_N=256 (number of FFT-bins) ==> log2(FFT_N) = 8     

// Table for bit-reversal process for 256-bin FFT
static const uint8_t armBitRevTable256[64] = {
    0x40, 0x20, 0x60, 0x10, 0x50, 0x30, 0x70, 0x08,
    0x48, 0x28, 0x68, 0x18, 0x58, 0x38, 0x78, 0x04,
    0x44, 0x24, 0x64, 0x14, 0x54, 0x34, 0x74, 0x0c,
    0x4c, 0x2c, 0x6c, 0x1c, 0x5c, 0x3c, 0x7c, 0x02,
    0x42, 0x22, 0x62, 0x12, 0x52, 0x32, 0x72, 0x0a,
    0x4a, 0x2a, 0x6a, 0x1a, 0x5a, 0x3a, 0x7a, 0x06,
    0x46, 0x26, 0x66, 0x16, 0x56, 0x36, 0x76, 0x0e,
    0x4e, 0x2e, 0x6e, 0x1e, 0x5e, 0x3e, 0x7e, 0x01 };

    
/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */


/*!
 *****************************************************************************
 * \brief In-place bit reversal function for 256-bin FFT.
 *
 * \param *pSrc - points to the complex input vector (processing occurs in-place)
 *****************************************************************************
 */
static void ams_bitreversal_256bin_int16(int16_t *pSrc16)
{
    int32_t *pSrc = (int32_t *)pSrc16;
    uint8_t *pBitRevTab = (uint8_t *)armBitRevTable256;
    uint16_t fftLen = 256;
    int32_t tmp;
    uint16_t fftLenBy2, fftLenBy2p1;
    uint16_t i, j;

    // Initialization
    j = 0;
    fftLenBy2 = fftLen / 2;
    fftLenBy2p1 = (fftLen / 2) + 1;

    // Bit-reversal
    for (i=0; i<=(fftLenBy2 - 2); i+=2)
    {
        if (i < j)
        {
            // pSrc[i] <-> pSrc[j];
            tmp = pSrc[i];
            pSrc[i] = pSrc[j];
            pSrc[j] = tmp;

            // pSrc[i + fftLenBy2p1] <-> pSrc[j + fftLenBy2p1];
            tmp = pSrc[i + fftLenBy2p1];
            pSrc[i + fftLenBy2p1] = pSrc[j + fftLenBy2p1];
            pSrc[j + fftLenBy2p1] = tmp;
        }

        // pSrc[i+1] <-> pSrc[j+fftLenBy2];
        tmp = pSrc[i + 1];
        pSrc[i + 1] = pSrc[j + fftLenBy2];
        pSrc[j + fftLenBy2] = tmp;

        // Reading the index for the bit reversal
        j = (uint16_t)(*pBitRevTab);

        // Updating the bit-reversal index depending on the fft length
        pBitRevTab += 1;
    }
}


/*!
 *****************************************************************************
 * \brief Inline 16-bit saturation
 *
 * \param x - the 32-bit value to saturate to 16-bits
 * \returns - the 16-bit saturated result
 *****************************************************************************
 */
static int32_t __SSAT_16(int32_t x)
{
    if (x > 32767)
    {
        x = 32767;
    }
    else if (x < -32768)
    {
        x = -32768;
    }
    return x;
}


// Radix-4 FFT algorithm...
//
// Input real and imaginary data:
//   x(n) = xa + j * ya
//   x(n+N/4) = xb + j * yb
//   x(n+N/2) = xc + j * yc
//   x(n+3N/4) = xd + j * yd
//
// Output real and imaginary data:
//   x(4r) = xa'+ j * ya'
//   x(4r+1) = xb'+ j * yb'
//   x(4r+2) = xc'+ j * yc'
//   x(4r+3) = xd'+ j * yd'
//
// Twiddle factors for radix-4 FFT:
//   Wn = cos1 + j * (-sin1)
//   W2n = cos2 + j * (-sin2)
//   W3n = cos3 + j * (-sin3)
//
// The real and imaginary output values for the radix-4 butterfly are
//   xa' = xa + xb + xc + xd
//   ya' = ya + yb + yc + yd
//   xb' = (xa+yb-xc-yd)*cos1 + (ya-xb-yc+xd)*sin1
//   yb' = (ya-xb-yc+xd)*cos1 - (xa+yb-xc-yd)*sin1
//   xc' = (xa-xb+xc-xd)*cos2 + (ya-yb+yc-yd)*sin2
//   yc' = (ya-yb+yc-yd)*cos2 - (xa-xb+xc-xd)*sin2
//   xd' = (xa-yb-xc+yd)*cos3 + (ya+xb-yc-xd)*sin3
//   yd' = (ya+xb-yc-xd)*cos3 - (xa-yb-xc+yd)*sin3
//

/*!
 *****************************************************************************
 * \brief Core function for the 16-bit CFFT butterfly process.
 * Internally the input is downscaled by 2 for every stage to avoid saturations
 * during CFFT processing.
 *
 * \param *pSrc - points to the complex input vector (processing occurs in-place)
 *****************************************************************************
 */
static void ams_radix4_butterfly_256bin_int16(int16_t *pSrc16)
{
    int16_t R0, R1, S0, S1, T0, T1, U0, U1;
    int16_t cos1, sin1, cos2, sin2, cos3, sin3, out1, out2;
    uint32_t n1, n2, ic, i0, i1, i2, i3, j, k;
    uint16_t fftLen = 256;
    uint16_t twidCoefModifier = 1;

    // Total processing is divided into three stages

    // Initialization for the first stage
    n2 = fftLen;
    n1 = n2;

    // n2 = fftLen/4
    n2 >>= 2;

    // Index for twiddle coefficient
    ic = 0;

    // Index for input read and output write
    i0 = 0;
    j = n2;

    // Input is in 1.15(q15) format

    // First stage processing...
    do
    {
        /* Butterfly implementation */

        /* index calculation for the inputs... */
        /* pSrc16[i0 + 0], pSrc16[i0 + fftLen/4], pSrc16[i0 + fftLen/2], pSrc16[i0 + 3fftLen/4] */
        i1 = i0 + n2;
        i2 = i1 + n2;
        i3 = i2 + n2;

        /* Reading i0, i0+fftLen/2 inputs */

        /* input is down scale by 4 to avoid overflow */
        /* Read ya(real), xa(imag) input */
        T0 = pSrc16[i0 * 2] >> 2;  // --1.13
        T1 = pSrc16[(i0 * 2) + 1] >> 2;  // --1.13

        /* input is down scale by 4 to avoid overflow */
        /* Read yc(real), xc(imag) input */
        S0 = pSrc16[i2 * 2] >> 2;  // --1.13
        S1 = pSrc16[(i2 * 2) + 1] >> 2;  // --1.13

        /* R0 = (ya + yc), R1 = (xa + xc) */
        R0 = (T0 + S0);  // -2.13
        R1 = (T1 + S1);  // -2.13

        /* S0 = (ya - yc), S1 = (xa - xc) */
        S0 = (T0 - S0);  // -2.13
        S1 = (T1 - S1);  // -2.13

        /* Reading i0+fftLen/4, i0+3fftLen/4 inputs */
        /* input is down scale by 4 to avoid overflow */
        /* Read yb(real), xb(imag) input */
        T0 = pSrc16[i1 * 2] >> 2;  // --1.13
        T1 = pSrc16[(i1 * 2) + 1] >> 2;  // --1.13

        /* input is down scale by 4 to avoid overflow */
        /* Read yd(real), xd(imag) input */
        U0 = pSrc16[i3 * 2] >> 2;  // --1.13
        U1 = pSrc16[(i3 * 2) + 1] >> 2;  // --1.13

        /* T0 = (yb + yd), T1 = (xb + xd) */
        T0 = (T0 + U0);  // -2.13
        T1 = (T1 + U1);  // -2.13

        /* writing the butterfly processed i0 sample */
        /* ya' = ya + yb + yc + yd */
        /* xa' = xa + xb + xc + xd */
        pSrc16[i0 * 2] = (R0 >> 1) + (T0 >> 1); // (--2.12) + (--2.12) ==> (-3.12)
        pSrc16[(i0 * 2) + 1] = (R1 >> 1) + (T1 >> 1); // (--2.12) + (--2.12) ==> (-3.12)

        /* R0 = (ya + yc) - (yb + yd) */
        /* R1 = (xa + xc) - (xb + xd) */
        R0 = (R0 - T0);  // 3.13
        R1 = (R1 - T1);  // 3.13

        /* cos2 & sin2 are read from Coefficient pointer */
        cos2 = twiddleCoef_i_int16_256[2 * ic + 64]; // 1.15
        sin2 = twiddleCoef_i_int16_256[2 * ic]; // 1.15

        /* xc' = (xa-xb+xc-xd) * cos2 + (ya-yb+yc-yd) * sin2 */
        /* yc' = (ya-yb+yc-yd) * cos2 - (xa-xb+xc-xd) * sin2 */
        out1 = (int16_t)((cos2 * R0 + sin2 * R1) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        out2 = (int16_t)((-sin2 * R0 + cos2 * R1) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12

        /* Reading i0+fftLen/4 */
        /* input is down scale by 4 to avoid overflow */
        /* T0 = yb, T1 = xb */
        T0 = pSrc16[i1 * 2] >> 2;  // --1.13
        T1 = pSrc16[(i1 * 2) + 1] >> 2;  // --1.13

        /* writing the butterfly processed i0 + fftLen/4 sample */
        /* writing output(xc', yc') in little endian cos1format */
        pSrc16[i1 * 2] = out1;  // 4.12
        pSrc16[(i1 * 2) + 1] = out2;  // 4.12

        /* Butterfly calculations */
        /* input is down scale by 4 to avoid overflow */
        /* U0 = yd, U1 = xd */
        U0 = pSrc16[i3 * 2] >> 2;  // --1.13
        U1 = pSrc16[(i3 * 2) + 1] >> 2;  // --1.13

        /* T0 = yb-yd, T1 = xb-xd */
        T0 = (T0 - U0);  // -2.13
        T1 = (T1 - U1);  // -2.13

        /* R1 = (ya-yc) + (xb- xd),  R0 = (xa-xc) - (yb-yd) */
        R0 = (S0 - T1);  // 3.13
        R1 = (S1 + T0);  // 3.13

        /* S1 = (ya-yc) - (xb- xd), S0 = (xa-xc) + (yb-yd) */
        S0 = (S0 + T1);  // 3.13
        S1 = (S1 - T0);  // 3.13

        /* cos1 & sin1 are read from Coefficient pointer */
        cos1 = twiddleCoef_i_int16_256[ic + 64]; // 1.15
        sin1 = twiddleCoef_i_int16_256[ic]; // 1.15

        /* Butterfly process for the i0+fftLen/2 sample */
        /* xb' = (xa+yb-xc-yd) * cos1 + (ya-xb-yc+xd) * sin1 */
        /* yb' = (ya-xb-yc+xd) * cos1 - (xa+yb-xc-yd) * sin1 */
        out1 = (int16_t)((sin1 * S1 + cos1 * S0) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        out2 = (int16_t)((-sin1 * S0 + cos1 * S1) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        /* writing output(xb', yb') in little endian format */
        pSrc16[i2 * 2] = out1;  // 4.12
        pSrc16[(i2 * 2) + 1] = out2;  // 4.12

        /* cos3 & sin3 are read from Coefficient pointer */
        if ((3 * ic) <= 128)
            cos3 = twiddleCoef_i_int16_256[3 * ic + 64]; // 1.15
        else
            cos3 = twiddleCoef_i_int16_256[256 - (3 * ic) + 64]; // 1.15
        sin3 = twiddleCoef_i_int16_256[3 * ic]; // 1.15

        /* Butterfly process for the i0+3fftLen/4 sample */
        /* xd' = (xa-yb-xc+yd) * cos3 + (ya+xb-yc-xd) * sin3 */
        /* yd' = (ya+xb-yc-xd) * cos3 - (xa-yb-xc+yd) * sin3 */
        out1 = (int16_t)((sin3 * R1 + cos3 * R0) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        out2 = (int16_t)((-sin3 * R0 + cos3 * R1) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        /* writing output(xd', yd') in little endian format */
        pSrc16[i3 * 2] = out1;  // 4.12
        pSrc16[(i3 * 2) + 1] = out2;  // 4.12

        /* Twiddle coefficients index modifier */
        ic = ic + twidCoefModifier;

        /* Updating input index */
        i0 = i0 + 1;

    } while(--j);
    // data is in 4.12(q12) format
    // end of first stage processing


    // Start of middle stage processing

    // Twiddle coefficients index modifier
    twidCoefModifier <<= 2;

    // Middle stage processing...
    for (k = fftLen / 4; k > 4; k >>= 2)
    {
        /* Initializations for the middle stage */
        n1 = n2;
        n2 >>= 2;
        ic = 0;

        for (j = 0; j <= (n2 - 1); j++)
        {
            /* index calculation for the coefficients */
            cos1 = twiddleCoef_i_int16_256[ic + 64]; // 1.15
            sin1 = twiddleCoef_i_int16_256[ic];
            cos2 = twiddleCoef_i_int16_256[2 * ic + 64];
            sin2 = twiddleCoef_i_int16_256[2 * ic];
            if ((3 * ic) <= 128)
                cos3 = twiddleCoef_i_int16_256[3 * ic + 64]; // 1.15
            else
                cos3 = twiddleCoef_i_int16_256[256 - (3 * ic) + 64]; // 1.15
            sin3 = twiddleCoef_i_int16_256[3 * ic];

            /* Twiddle coefficients index modifier */
            ic = ic + twidCoefModifier;

            /* Butterfly implementation */
            for (i0 = j; i0 < fftLen; i0 += n1)
            {
                /* index calculation for the inputs... */
                /* pSrc16[i0 + 0], pSrc16[i0 + fftLen/4], pSrc16[i0 + fftLen/2], pSrc16[i0 + 3fftLen/4] */
                i1 = i0 + n2;
                i2 = i1 + n2;
                i3 = i2 + n2;

                /* Reading i0, i0+fftLen/2 inputs */
                /* Read ya(real), xa(imag) input */
                T0 = pSrc16[i0 * 2];  // 4.12
                T1 = pSrc16[(i0 * 2) + 1];  // 4.12

                /* Read yc(real), xc(imag) input */
                S0 = pSrc16[i2 * 2];  // 4.12
                S1 = pSrc16[(i2 * 2) + 1];  // 4.12

                /* R0 = (ya + yc), R1 = (xa + xc) */
                R0 = __SSAT_16((int32_t)T0 + S0);  // 4.12
                R1 = __SSAT_16((int32_t)T1 + S1);  // 4.12

                /* S0 = (ya - yc), S1 =(xa - xc) */
                S0 = __SSAT_16((int32_t)T0 - S0);  // 4.12
                S1 = __SSAT_16((int32_t)T1 - S1);  // 4.12

                /* Reading i0+fftLen/4, i0+3fftLen/4 inputs */
                /* Read yb(real), xb(imag) input */
                T0 = pSrc16[i1 * 2];  // 4.12
                T1 = pSrc16[(i1 * 2) + 1];  // 4.12

                /* Read yd(real), xd(imag) input */
                U0 = pSrc16[i3 * 2];  // 4.12
                U1 = pSrc16[(i3 * 2) + 1];  // 4.12

                /* T0 = (yb + yd), T1 = (xb + xd) */
                T0 = __SSAT_16((int32_t)T0 + U0);  // 4.12
                T1 = __SSAT_16((int32_t)T1 + U1);  // 4.12

                /* writing the butterfly processed i0 sample */

                /* xa' = xa + xb + xc + xd */
                /* ya' = ya + yb + yc + yd */
                out1 = ((R0 >> 1) + (T0 >> 1)) >> 1;  // 6.10
                out2 = ((R1 >> 1) + (T1 >> 1)) >> 1;  // 6.10

                pSrc16[i0 * 2] = out1;  // 6.10
                pSrc16[(i0 * 2) + 1] = out2;  // 6.10

                /* R0 = (ya + yc) - (yb + yd), R1 = (xa + xc) - (xb + xd) */
                R0 = (R0 >> 1) - (T0 >> 1);  // 5.11
                R1 = (R1 >> 1) - (T1 >> 1);  // 5.11

                /* (ya-yb+yc-yd) * sin2 + (xa-xb+xc-xd) * cos2 */
                /* (ya-yb+yc-yd) * cos2 - (xa-xb+xc-xd) * sin2 */
                out1 = (int16_t)((cos2 * R0 + sin2 * R1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10
                out2 = (int16_t)((-sin2 * R0 + cos2 * R1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10

                /* Reading i0+3fftLen/4 */
                /* Read yb(real), xb(imag) input */
                T0 = pSrc16[i1 * 2];  // 4.12
                T1 = pSrc16[(i1 * 2) + 1];  // 4.12

                /* writing the butterfly processed i0 + fftLen/4 sample */
                /* xc' = (xa-xb+xc-xd) * cos2 + (ya-yb+yc-yd) * sin2 */
                /* yc' = (ya-yb+yc-yd) * cos2 - (xa-xb+xc-xd) * sin2 */
                pSrc16[i1 * 2] = out1;  // 6.10
                pSrc16[(i1 * 2) + 1] = out2;  // 6.10

                /* Butterfly calculations */

                /* Read yd(real), xd(imag) input */
                U0 = pSrc16[i3 * 2];  // 4.12
                U1 = pSrc16[(i3 * 2) + 1];  // 4.12

                /* T0 = yb-yd, T1 = xb-xd */
                T0 = __SSAT_16((int32_t)T0 - U0);  // 4.12
                T1 = __SSAT_16((int32_t)T1 - U1);  // 4.12

                /* R0 = (ya-yc) + (xb- xd), R1 = (xa-xc) - (yb-yd) */
                R0 = (S0 >> 1) - (T1 >> 1);  // 5.11
                R1 = (S1 >> 1) + (T0 >> 1);  // 5.11

                /* S0 = (ya-yc) - (xb- xd), S1 = (xa-xc) + (yb-yd) */
                S0 = (S0 >> 1) + (T1 >> 1);  // 5.11
                S1 = (S1 >> 1) - (T0 >> 1);  // 5.11

                /* Butterfly process for the i0+fftLen/2 sample */
                out1 = (int16_t)((cos1 * S0 + sin1 * S1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10
                out2 = (int16_t)((-sin1 * S0 + cos1 * S1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10

                /* xb' = (xa+yb-xc-yd) * cos1 + (ya-xb-yc+xd) * sin1 */
                /* yb' = (ya-xb-yc+xd) * cos1 - (xa+yb-xc-yd) * sin1 */
                pSrc16[i2 * 2] = out1;  // 6.10
                pSrc16[(i2 * 2) + 1] = out2;  // 6.10

                /* Butterfly process for the i0+3fftLen/4 sample */
                out1 = (int16_t)((sin3 * R1 + cos3 * R0) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10
                out2 = (int16_t)((-sin3 * R0 + cos3 * R1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10

                /* xd' = (xa-yb-xc+yd) * cos3 + (ya+xb-yc-xd) * sin3 */
                /* yd' = (ya+xb-yc-xd) * cos3 - (xa-yb-xc+yd) * sin3 */
                pSrc16[i3 * 2] = out1;  // 6.10
                pSrc16[(i3 * 2) + 1] = out2;  // 6.10
            }
        }
        /* Twiddle coefficients index modifier */
        twidCoefModifier <<= 2;
    }
    // end of middle stage processing


    // data is in 10.6(q6) format for the 1024 point
    // data is in 8.8(q8) format for the 256 point
    // data is in 6.10(q10) format for the 64 point
    // data is in 4.12(q12) format for the 16 point

    // Initialization for the last stage
    n1 = n2;
    n2 >>= 2;

    // Last stage processing...

    /* Butterfly implementation */
    for (i0 = 0; i0 <= (fftLen - n1); i0 += n1)
    {
        /* index calculation for the inputs... */
        /* pSrc16[i0 + 0], pSrc16[i0 + fftLen/4], pSrc16[i0 + fftLen/2], pSrc16[i0 + 3fftLen/4] */
        i1 = i0 + n2;
        i2 = i1 + n2;
        i3 = i2 + n2;

        /* Reading i0, i0+fftLen/2 inputs */
        /* Read ya (real), xa(imag) input */
        T0 = pSrc16[i0 * 2];  // 8.8
        T1 = pSrc16[(i0 * 2) + 1];  // 8.8

        /* Read yc(real), xc(imag) input */
        S0 = pSrc16[i2 * 2];  // 8.8
        S1 = pSrc16[(i2 * 2) + 1];  // 8.8

        /* R0 = (ya + yc), R1 = (xa + xc) */
        R0 = __SSAT_16((int32_t)T0 + S0);  // 8.8
        R1 = __SSAT_16((int32_t)T1 + S1);  // 8.8

        /* S0 = (ya - yc), S1 = (xa - xc) */
        S0 = __SSAT_16((int32_t)T0 - S0);  // 8.8
        S1 = __SSAT_16((int32_t)T1 - S1);  // 8.8

        /* Reading i0+fftLen/4, i0+3fftLen/4 inputs */
        /* Read yb (real), xb(imag) input */
        T0 = pSrc16[i1 * 2];  // 8.8
        T1 = pSrc16[(i1 * 2) + 1];  // 8.8

        /* Read yd (real), xd(imag) input */
        U0 = pSrc16[i3 * 2];  // 8.8
        U1 = pSrc16[(i3 * 2) + 1];  // 8.8

        /* T0 = (yb + yd), T1 = (xb + xd) */
        T0 = __SSAT_16((int32_t)T0 + U0);  // 8.8
        T1 = __SSAT_16((int32_t)T1 + U1);  // 8.8

        /* writing the butterfly processed i0 sample */
        /* xa' = (xa+xb+xc+xd) */
        /* ya' = (ya+yb+yc+yd) */
        pSrc16[i0 * 2] = (R0 >> 1) + (T0 >> 1);  // 9.7
        pSrc16[(i0 * 2) + 1] = (R1 >> 1) + (T1 >> 1);  // 9.7

        /* R0 = (ya + yc) - (yb + yd), R1 = (xa + xc) - (xb + xd) */
        R0 = (R0 >> 1) - (T0 >> 1);  // 9.7
        R1 = (R1 >> 1) - (T1 >> 1);  // 9.7

        /* Read yb(real), xb(imag) input */
        T0 = pSrc16[i1 * 2];  // 8.8
        T1 = pSrc16[(i1 * 2) + 1];  // 8.8

        /* writing the butterfly processed i0 + fftLen/4 sample */
        /* xc' = (xa-xb+xc-xd) */
        /* yc' = (ya-yb+yc-yd) */
        pSrc16[i1 * 2] = R0;  // 9.7
        pSrc16[(i1 * 2) + 1] = R1;  // 9.7

        /* Read yd(real), xd(imag) input */
        U0 = pSrc16[i3 * 2];  // 8.8
        U1 = pSrc16[(i3 * 2) + 1];  // 8.8

        /* T0 = (yb - yd), T1 = (xb - xd)  */
        T0 = __SSAT_16((int32_t)T0 - U0);  // 8.8
        T1 = __SSAT_16((int32_t)T1 - U1);  // 8.8

        /* writing the butterfly processed i0 + fftLen/2 sample */
        /* xb' = (xa+yb-xc-yd) */
        /* yb' = (ya-xb-yc+xd) */
        pSrc16[i2 * 2] = (S0 >> 1) + (T1 >> 1);  // 9.7
        pSrc16[(i2 * 2) + 1] = (S1 >> 1) - (T0 >> 1);  // 9.7

        /* writing the butterfly processed i0 + 3fftLen/4 sample */
        /* xd' = (xa-yb-xc+yd) */
        /* yd' = (ya+xb-yc-xd) */
        pSrc16[i3 * 2] = (S0 >> 1) - (T1 >> 1);  // 9.7
        pSrc16[(i3 * 2) + 1] = (S1 >> 1) + (T0 >> 1);  // 9.7
    }

    // end of last stage processing

    // output is in 11.5(q5) format for the 1024 point
    // output is in 9.7(q7) format for the 256 point
    // output is in 7.9(q9) format for the 64 point
    // output is in 5.11(q11) format for the 16 point
}


/*!
 *****************************************************************************
 * \brief Processing function for the 16-bit 256-bin CFFT
 * Internally the input is downscaled by 2 for every stage to avoid saturations
 * during CFFT processing.
 *
 * \param *pSrc - points to the complex input vector (processing occurs in-place)
 *****************************************************************************
 */
void ams_cfft_radix4_int16_256(int16_t *pSrc)
{
    // Complex-FFT radix-4
    ams_radix4_butterfly_256bin_int16(pSrc);

    // Bit Reversal
    ams_bitreversal_256bin_int16(pSrc);
}


/*!
 *****************************************************************************
 * \brief Core Real-FFT process
 * Computes the first 256-bins of 512-bin FFT using the 256-bin result of complex-FFT.
 * Ar=[0..1], Ai=[-0.5..0.5], Br=[0..1], Bi=[-0.5..0.5]
 * Br=(1.0-Ar)=(0x8000-Ar), Bi=(-Ai)
 * Ar[(N/2)-i]=Ar[i], Ai[(N/2)-i]=-Ai[i]
 * Ai[i]=Ar[(N/4)-i]-0x4000
 *
 * \param *pSrc - array of 256 complex-values (processing occurs in-place)
 *****************************************************************************
 */
void ams_split_in_place_rfft_512bin_int16(int16_t *pSrc)
{
    uint16_t i;
    int32_t outR1, outI1, outR2, outI2;
    int16_t coefAr, coefAi;
    int16_t *pSrc1, *pSrc2;
    #define FFT_LEN_RFFT 512
    #define FFT_LEN_SPLIT_IN_PLACE (FFT_LEN_RFFT>>1)

    // handle all cases except "i=N/4" and "i=0"
    for (i=1; i<(FFT_LEN_SPLIT_IN_PLACE>>1); i++)
    {
        // setup data-pointers for "i" and "N/2-i"
        pSrc1 = &pSrc[i * 2];
        pSrc2 = &pSrc[(FFT_LEN_SPLIT_IN_PLACE - i) * 2];

        // setup twiddle-coefficients for "i"
        coefAr = twiddleCoef_Ar_int16_256[i];
        coefAi = twiddleCoef_Ar_int16_256[(FFT_LEN_SPLIT_IN_PLACE>>1)-i] - 0x4000;
 
        // outR[i] = (pSrc[2i] * pATable[2i] - pSrc[2i + 1] * pATable[2i + 1] +
        //            pSrc[2(N-i)] * pBTable[2i] + pSrc[2(N-i) + 1] * pBTable[2i + 1]);
        outR1 = (*pSrc1 * coefAr) - (*(pSrc1 + 1) * coefAi);
        outR1 += (*pSrc2 * (0x8000 - coefAr)) + (*(pSrc2 + 1) * (-coefAi));

        // outI[i] = (pSrc[2i + 1] * pATable[2i] + pSrc[2i] * pATable[2i + 1] +
        //            pSrc[2(N-i)] * pBTable[2i + 1] - pSrc[2(N-i) + 1] * pBTable[2i]);
        outI1 = (*(pSrc1 + 1) * coefAr) + (*pSrc1 * coefAi);
        outI1 += (*pSrc2 * (-coefAi)) - (*(pSrc2 + 1) * (0x8000 - coefAr));

        // setup twiddle-coefficients for "N/2-i"
        // coefAr = pATable[(FFT_LEN_SPLIT_IN_PLACE - i) * 2] = pATable[i * 2] {same as above}
        // coefAi = pATable[(FFT_LEN_SPLIT_IN_PLACE - i) * 2 + 1] = -pATable[i * 2 + 1] {negaitive of above}
        coefAi = -coefAi;
 
        // outR[N-i] = (pSrc[2(N-i)] * pATable[2(N-i)] - pSrc[2(N-i) + 1] * pATable[2(N-i) + 1] +
        //              pSrc[2i] * pBTable[2(N-i)] + pSrc[2i + 1] * pBTable[2(N-i) + 1]);
        outR2 = (*pSrc2 * coefAr) - (*(pSrc2 + 1) * coefAi);
        outR2 += (*pSrc1 * (0x8000 - coefAr)) + (*(pSrc1 + 1) * (-coefAi));

        // outI[N-i] = (pSrc[2(N-i) + 1] * pATable[2(N-i)] + pSrc[2(N-i)] * pATable[2(N-i) + 1] +
        //              pSrc[2i] * pBTable[2(N-i) + 1] - pSrc[2i + 1] * pBTable[2(N-i)]);
        outI2 = (*(pSrc2 + 1) * coefAr) + (*pSrc2 * coefAi);
        outI2 += (*pSrc1 * (-coefAi)) - (*(pSrc1 + 1) * (0x8000 - coefAr));

        // write output for "i" and "N/2-i"
        *pSrc1 = (int16_t)(outR1 >> 15);
        *(pSrc1+1) = (int16_t)(outI1 >> 15);

        *pSrc2 = (int16_t)(outR2 >> 15);
        *(pSrc2+1) = (int16_t)(outI2 >> 15);
    }

    // handle "i=N/4" (note:  N/4 == N/2 - N/4)
    // write output for "i = N/4"
    // pSrc[(N/4) * 2] is already correct as-is
    pSrc1 = &pSrc[i * 2 + 1]; // setup data-pointers for "i = N/4" imaginary-component
    *pSrc1 = -(*pSrc1); // pSrc[(N/4) * 2 + 1] = -pSrc[(N/4) * 2 + 1]
    
    // handle "i=0"
    pSrc[0] = pSrc[0] + pSrc[1];
    pSrc[1] = 0;
}


/*!
 *****************************************************************************
 * \brief Computes the 512-bin FFT for the 512-value real-input-array
 *
 * \param *pSrc - 512-value real-input-array (processing occurs in-place)
 *****************************************************************************
 */
void ams_rfft_int16_512(int16_t *pSrc)
{
    // Perform a 256-bin complex FFT using the real-data as input
    ams_radix4_butterfly_256bin_int16(pSrc);
    ams_bitreversal_256bin_int16(pSrc);

    // Compute the first 256-bins of the 512-bin result
    ams_split_in_place_rfft_512bin_int16(pSrc);
}


/*!
 *****************************************************************************
 * \brief Computes the complex magnitude for the complex-array
 *
 * \param *pSrc - points to the complex input vector
 * \param *pDst - points to the real output vector [magnitude]
 * \param numSamples - number of complex samples in the input vector 
 *****************************************************************************
 */
void complexMagnitude(int16_t *pSrc, uint16_t *pDst, uint16_t numSamples)
{
    int16_t real, imag;
    uint32_t magSq;

    while (numSamples > 0)
    {
        // result = sqrt((real * real) + (imag * imag))
        real = *pSrc++;
        imag = *pSrc++;
        magSq = (uint32_t)(((int32_t)real * real) + ((int32_t)imag * imag));
        *pDst++ = (uint16_t)sqrtf((float)magSq);

        // decrement the loop counter
        numSamples--;
    }
}
