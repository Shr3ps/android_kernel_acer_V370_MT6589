/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*******************************************************************************
 *
 * Filename:
 * ---------
 * audio_acf_default.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 * This file is the header of audio customization related parameters or definition.
 *
 * Author:
 * -------
 * Tina Tsai
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#ifndef AUDIO_ACF_DEFAULT_H
#define AUDIO_ACF_DEFAULT_H

    /* Compensation Filter HSF coeffs: default all pass filter       */
    /* BesLoudness also uses this coeffs    */ 
    #define BES_LOUDNESS_HSF_COEFF \
    0x07DA170A, 0xF054253E, 0x07D1C95A, 0x7D56C298, 0x00000000, \
	0x07D6C2CB, 0xF05B867E, 0x07CDBD5F, 0x7D19C2D2, 0x00000000, \
	0x07C751CE, 0xF07DBC74, 0x07BAFE49, 0x7BFFC3DA, 0x00000000, \
	0x07B4AD39, 0xF0A6FF56, 0x07A46981, 0x7AA6C515, 0x00000000, \
	0x07AE1CEE, 0xF0B5836A, 0x079C79B2, 0x7A2CC583, 0x00000000, \
	0x078FC91E, 0xF0F882BF, 0x0777E4C0, 0x77F1C777, 0x00000000, \
	0x076B705A, 0xF148A5E6, 0x074C3E63, 0x7538C9C0, 0x00000000, \
	0x075EB726, 0xF164A91C, 0x073D0346, 0x7441CA88, 0x00000000, \
	0x07247309, 0xF1E4AE1A, 0x06F7957D, 0x6FC0CE05, 0x00000000, \
\
	0x08000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x08000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x08000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x08000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x08000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x08000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x08000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x08000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x08000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 
   

    /* Compensation Filter BPF coeffs: default all pass filter      */ 
    #define BES_LOUDNESS_BPF_COEFF \
    0x3FD481A8,0x3EFF7E57,0xC12C0000, \ 
    0x3FD081DA,0x3EE97E25,0xC1460000, \ 
    0x3FBE82D7,0x3E817D28,0xC1C00000, \ 
    0x3FA98440,0x3E037BBF,0xC2520000, \ 
    0x3FA184CE,0x3DD77B31,0xC2860000, \ 
    0x3F7E87BD,0x3D0C7842,0xC3740000, \ 
\
    0x3FD481C0,0x3EFF7E3F,0xC12C0000, \ 
    0x3FD081F5,0x3EE97E0A,0xC1460000, \ 
    0x3FBE830B,0x3E817CF4,0xC1C00000, \ 
    0x3FA9849C,0x3E037B63,0xC2520000, \ 
    0x3FA1853A,0x3DD77AC5,0xC2860000, \ 
    0x3F7E8889,0x3D0C7776,0xC3740000, \ 
\
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
\    
 	0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \         
\    
 	0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \     
\    
 	0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \         
\    
 	0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \    
\
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000, \ 
    0x40000000,0x00000000,0x00000000     
    
    #define BES_LOUDNESS_LPF_COEFF \
    0x40000000, 0x00000000, 0x00000000,\ 
	0x40000000, 0x00000000, 0x00000000,\ 
	0x40000000, 0x00000000, 0x00000000,\ 
	0x40000000, 0x00000000, 0x00000000,\ 
	0x40000000, 0x00000000, 0x00000000,\ 
	0x40000000, 0x00000000, 0x00000000 

    #define BES_LOUDNESS_WS_GAIN_MAX  0
           
    #define BES_LOUDNESS_WS_GAIN_MIN  0
           
    #define BES_LOUDNESS_FILTER_FIRST  0
           
    #define BES_LOUDNESS_GAIN_MAP_IN \
    0, 0, 0, 0,  0
   
    #define BES_LOUDNESS_GAIN_MAP_OUT \            
    0, 0, 0, 0, 0

	#define BES_LOUDNESS_ATT_TIME	164
	#define BES_LOUDNESS_REL_TIME	16400              

#endif
