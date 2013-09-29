#include  "stm32f4xx.h"
#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include  "SMS_PDU.h"

const char GB_DATA[]="京津沪宁渝琼藏川粤青贵闽吉陕蒙晋甘桂鄂赣浙苏新鲁皖湘黑辽云豫冀";
const char UCS2_CODE[]="4EAC6D256CAA5B816E1D743C85CF5DDD7CA497528D3595FD540996558499664B7518684291028D636D5982CF65B09C8176966E589ED18FBD4E918C6B5180";


/*获取一个指定字符的位置，中文字符作为一个字符计算*/
static int StringFind(const char* string,const char* find,int number)
{
	char* pos;
	int count = 0;
	//pos=string;
	//p = string;
	pos=strstr(string,find);
	if (pos == (void *)0)
		return -1;
	count=pos-string;
	return count;
	#ifdef 0
	while (number > 0)
	{
		/*定义查找到的字符位置的指针，以便临时指针进行遍历*/
		pos = strstr(p,find);
		/*当位置指针为0时，说明没有找到这个字符*/
		if (pos == (void *)0)
		return -1;
		/*当位置指针和临时指针相等说明下一个字符就是要找的字符，如果临时指针小于位置指针，则进行遍历字符串操作，并将count增1*/
		while(p <= pos)
		{
			if(*p > 0x80 || *p < 0)
			{
				p++;
			}
			p++;
			count++;
		}
		/*对要查找的次数减一*/
		number--;
	}
	return count;
	#endif
}


u16 Hex_To_Ascii(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	
	const u8 tab[]="0123456789ABCDEF";    // 0x0-0xf的字符查找表
	u16 i;
    
    for( i=0; i<nSrcLength; i++)
    {
		// 输出低4位
        *pDst++ = tab[*pSrc >> 4];
                                        
        // 输出高4位
        *pDst++ = tab[*pSrc & 0x0f];
    
        pSrc++;
	}
    
    // 输出字符串加个结束符
    *pDst = '\0';
    
    // 返回目标字符串长度
    return (nSrcLength << 1);
}


u16 Ascii_To_Hex(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	u16 i;
	for(i=0; i<nSrcLength; i+=2)
    {
        // 输出高4位
        if(*pSrc>='0' && *pSrc<='9')
        {
            *pDst = (*pSrc - '0') << 4;
		}
        else if(*pSrc>='A' && *pSrc<='F')
        {
            *pDst = (*pSrc - 'A' + 10) << 4;
        }
        else  if(*pSrc>='a' && *pSrc<='f')
        {
             *pDst = (*pSrc - 'a' + 10) << 4;
        
        }
        else
           return FALSE;
        pSrc++;
      // 输出低4位
        if(*pSrc>='0' && *pSrc<='9')
        {
            *pDst |= *pSrc - '0';
        }
        else if(*pSrc>='A' && *pSrc<='F')
		{
            *pDst |= *pSrc - 'A' + 10;
        }
         else  if(*pSrc>='a' && *pSrc<='f')
        {
             *pDst |= (*pSrc - 'a' + 10);
        
        }
        else
           return FALSE;
        pSrc++;
        pDst++;
    }
    
    // 返回目标数据长度
    return (nSrcLength >> 1);	
}



u16 GsmDecode8bit(const u8 *pSrc, u8 *pDst, u16 nSrcLength)
{
	u16 m;
	// 简单复制
	for(m=0;m<nSrcLength;m++)
	   *pDst++=*pSrc++;
	// 输出字符串加个结束符
	*pDst = '\0';
	return nSrcLength;
}

u16  GsmEncode8bit(const u8 *pSrc, u8 *pDst, u16 nSrcLength)
{
	u16 m;
	// 简单复制
	for(m=0;m<nSrcLength;m++)
	   *pDst++=*pSrc++;

	return nSrcLength;
}

u16 GsmDecodeUcs2(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	u16 nDstLength=0;        // UNICODE宽字符数目
	u16 i;
	s16 indexNum;
	char strTemp[5];
	const u8 *p;
	// INT16U wchar[128];      // UNICODE串缓冲区

	rt_kprintf("\r\n GsmDecodeUcs2");
    // 高低字节对调，拼成UNICODE
    for(i=0; i<nSrcLength; i+=2)
    {
		if(*pSrc)		///汉字编码
    		{
    		p=pSrc;
			Hex_To_Ascii(p,strTemp,2);
			strTemp[4]=0;
    		indexNum=StringFind(UCS2_CODE,strTemp,strlen(UCS2_CODE));
			rt_kprintf("\r\n index=%d,str=%s",indexNum,strTemp);
			if(indexNum>=0)
				{
				indexNum=indexNum>>1;
				*pDst++ = GB_DATA[indexNum];
				*pDst++ = GB_DATA[1+indexNum];
				}
			else
				{
				*pDst++ = 0x20;		///不可识别的汉子用"※"表示
				*pDst++ = 0x3B;
				//*pDst++ = *pSrc;		///不可识别的汉子用UCS2源码表示
				//*pDst++ = *(pSrc+1);
				}
			pSrc+=2;
			nDstLength+=2;
    		}
		else						///英文编码
			{
			
			// 先高位字节,因为是数据。高字节为0
	        pSrc++;
	        // 后低位字节
	        *pDst++= *pSrc++;
			nDstLength++;
			}
   
    }
        // UNICODE串-.字符串
    //nDstLength = ::WideCharToMultiByte(CP_ACP, 0, wchar, nSrcLength/2, pDst, 160, NULL, NULL);
        // 输出字符串加个结束符    
    //pDst[nDstLength] = '\0';    
        // 返回目标字符串长度
    return nDstLength;
}


u16 GsmEncodeUcs2(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	u16 nDstLength=nSrcLength;        // UNICODE宽字符数目
	u16 i;
	s16 indexNum;
	char strTemp[3];
    //INT16U wchar[128];      // UNICODE串缓冲区
    nDstLength=0;
    // 字符串-.UNICODE串
   // nDstLength = ::MultiByteToWideChar(CP_ACP, 0, pSrc, nSrcLength, wchar, 128);
 
    // 高低字节对调，输出
    for(i=0; i<nSrcLength; i++)
    {
    	if((*pSrc&0x80)==0x80)		///汉字编码
    		{
    		strTemp[0]=*pSrc;
    		strTemp[1]=*(pSrc+1);
			strTemp[2]=0;
    		indexNum=StringFind(GB_DATA,strTemp,strlen(GB_DATA));
			if(indexNum>=0)
				{
				indexNum=indexNum*2;
				Ascii_To_Hex(&UCS2_CODE[indexNum],strTemp,4);
				*pDst++ = strTemp[0];
				*pDst++ = strTemp[1];
				}
			else		///不可识别的汉子用"※"表示
				{
				*pDst++ = 0x20;
				*pDst++ = 0x3B;
				}
			pSrc+=2;
			i++;
    		}
		else						///英文编码
			{
	         // 先输出高位字节
	        *pDst++ = 0x00;
	        // 后输出低位字节
	        *pDst++ = * pSrc++;
			}
		nDstLength++;
       
    }
    
    // 返回目标编码串长度
    return (nDstLength <<1);
}


u16 GsmDecode7bit(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
    u16 nSrc;        // 源字符串的计数值
    u16 nDst;        // 目标解码串的计数值
    u16 nByte;       // 当前正在处理的组内字节的序号，范围是0-6
    u8 nLeft;    // 上一字节残余的数据
    
    // 计数值初始化
    nSrc = 0;
    nDst = 0;
    
    // 组内字节序号和残余数据初始化
    nByte = 0;
    nLeft = 0;
    
    // 将源数据每7个字节分为一组，解压缩成8个字节
  // 循环该处理过程，直至源数据被处理完
    // 如果分组不到7字节，也能正确处理
    while(nSrc<nSrcLength)
    {
        // 将源字节右边部分与残余数据相加，去掉最高位，得到一个目标解码字节
        *pDst = ((*pSrc << nByte) | nLeft) & 0x7f;
        // 将该字节剩下的左边部分，作为残余数据保存起来
        nLeft = *pSrc >> (7-nByte);
    
        // 修改目标串的指针和计数值
        pDst++;
        nDst++;
   // 修改字节计数值
        nByte++;
    
        // 到了一组的最后一个字节
        if(nByte == 7)
        {
            // 额外得到一个目标解码字节
            *pDst = nLeft;
    
            // 修改目标串的指针和计数值
            pDst++;
            nDst++;
    
            // 组内字节序号和残余数据初始化
            nByte = 0;
            nLeft = 0;
        }
    
        // 修改源串的指针和计数值
        pSrc++;
        nSrc++;
	}
    
    *pDst = 0;
    
    // 返回目标串长度
    return nDst;

	
}
//将每个ascii8位编码的Bit8去掉，依次将下7位编码的后几位逐次移到前面，形成新的8位编码。 
u16 GsmEncode7bit(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
    u16 nSrc;        // 源字符串的计数值
    u16 nDst;        // 目标编码串的计数值
    u16 nChar;       // 当前正在处理的组内字符字节的序号，范围是0-7
    u8 nLeft;    // 上一字节残余的数据
    
    // 计数值初始化
    nSrc = 0;
    nDst = 0;
    
    // 将源串每8个字节分为一组，压缩成7个字节
    // 循环该处理过程，直至源串被处理完
    // 如果分组不到8字节，也能正确处理
    while(nSrc<nSrcLength+1)
    {
        // 取源字符串的计数值的最低3位
        nChar = nSrc & 7;
		  // 处理源串的每个字节
        if(nChar == 0)
        {
            // 组内第一个字节，只是保存起来，待处理下一个字节时使用
            nLeft = *pSrc;
        }
        else
        {
            // 组内其它字节，将其右边部分与残余数据相加，得到一个目标编码字节
            *pDst = (*pSrc << (8-nChar)) | nLeft;
      // 将该字节剩下的左边部分，作为残余数据保存起来
            nLeft = *pSrc >> nChar;
            // 修改目标串的指针和计数值 pDst++;
            pDst++;
            nDst++; 
        } 
        
        // 修改源串的指针和计数值
        pSrc++; nSrc++;
    }
    
    // 返回目标串长度
    return nDst; 
	
}




// 两两颠倒的字符串转换为正常顺序的字符串
// 如："8613693092030" -. "683196032930F0"
// pSrc: 源字符串指针
// pDst: 目标字符串指针
// nSrcLength: 源字符串长度
// 返回: 目标字符串长度
u8 Hex_Num_Encode(const u8 *pSrc,u8 *pDst,u8 nSrcLength)
{
  u8 nDstLength=nSrcLength;
	u8 i;
  if(nDstLength&0x01)
    nDstLength+=1;
  for(i=0;i<nDstLength;i+=2)
  {
   *pDst=(*pSrc<<4)|(*pSrc>>4);
    pDst++;
    pSrc++;
  }
  if(nSrcLength&1)
    {
     *(pDst-1)&=0x0f;
     *(pDst-1)|=0xf0;
    }
   return (nDstLength>>1);
}

// 两两颠倒的字符串转换为正常顺序的字符串
// 如："8613693092030" -. "683196032930F0"
// pSrc: 源字符串指针
// pDst: 目标字符串指针
// nSrcLength: 源字符串长度
// 返回: 目标字符串长度
u8 Hex_Num_Decode(const u8 *pSrc,u8 *pDst,u8 nSrcLength)
{
  u8 nDstLength=nSrcLength;
  u8 i;
  if(nDstLength&0x01)
    nDstLength+=1;
  for( i=0;i<nDstLength;i+=2)
  {
    *pDst=(*pSrc<<4)|(*pSrc>>4);
    pDst++;
    pSrc++;
  }
  if(nSrcLength&1)
    {
     *(pDst-1)&=0xf0;
     *(pDst-1)|=0x0f;
    }
  return (nDstLength>>1);
}


//返回号码长度
u8 Que_Number_Length(const u8 *Src)
  {
     u8 n=0;
	 u8 m;
     for(m=0;m<8;m++)
     {
       if((Src[m]&0xf0)==0xf0)
        break;
        n++;
      if((Src[m]&0x0f)==0x0f)
         break;
         n++;
     }
     return n;
  
  }


/*********************************************************************************
*函数名称:u16 SetPhoneNumToPDU(u8 *pDest,u8 *pSrc,u16 len)
*功能描述:将字符串格式转换成BCD码格式的PDU电话号码，如原始号码为"13820554863"转为13820554863F
*输    入:pSrc 原始数据，len输出的最大长度
*输    出:pDest 输出字符串
*返 回 值:	有效的号码长度
*作    者:白养民
*创建日期:2013-05-28
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u16 SetPhoneNumToPDU(u8 *pDest,char *pSrc,u16 len)
{
	u16 i;
	
    memset(pDest,0xff,len);
    for( i=0; i<len; i++)
    {
		if((*pSrc>=0x30)&&(*pSrc<=0x39))
			{
			if(i%2==0)
				{
				*pDest &=0x0F;
				*pDest |= ((*pSrc-0x30)<<4);
				}
			else
				{
				*pDest &=0xF0;
				*pDest |= *pSrc-0x30;
				pDest++;
				}
			}
		else
			{
			return i;
			}
		pSrc++;
	}
	return i;
}
/*********************************************************************************
*函数名称:void GetPhoneNumFromPDU(u8 *pDest,u8 *pSrc,u16 len)
*功能描述:将BCD码格式的PDU电话号码转换成字符串格式，如原始号码为13820554863F转为"13820554863"
*输    入:pSrc 原始数据，len输入的最大长度
*输    出:pDest 输出字符串
*返 回 值:	有效的号码长度
*作    者:白养民
*创建日期:2013-05-28
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u16 GetPhoneNumFromPDU(char *pDest,u8 *pSrc,u16 len)
{
	u16 i,j;
	for(j=0,i=0;i<len;i++)
		{
		if((pSrc[i]&0x0f)!=0x0f)
			{
			pDest[j++]=(pSrc[i]>>4)+0x30;
			pDest[j++]=(pSrc[i]&0x0f)+0x30;
			}
		else
			{
			pDest[j++]=(pSrc[i]>>4)+0x30;
			pDest[j]=0;
			break;
			}
		}
	return j;
}

//ok_bym
//解码程序
u16   GsmDecodePdu(const u8* pSrc,u16 pSrcLength,SmsType *pSmstype,u8 *DataDst)
{
	u8 nDstLength=0;          // 目标PDU串长度
	u8 tmp;       // 内部用的临时字节变量
	u8 buf[256];  // 内部用的缓冲区
	u16 templen=0;
	u16 tmp16;
    ///0891683108200245F320 0D90 683128504568F3 0008315032419430235E00540057003700300033002300440045005600490043004500490044002800310033003300340035003600370034003800360033002900230049005000310028003100320035002E00330038002E003100380032002E0036003000296D25
    //---SCA
    // SMSC地址信息段
    Ascii_To_Hex(pSrc, &tmp, 2);    // 取长度
    if(tmp>0&&tmp<=12)
    {
    
    tmp = (tmp - 1) * 2;    // SMSC号码串长度,去掉了91;
    pSrc += 4;              // 指针后移,长度两个字节，91两个字节。共4个字节
    templen+=4;
    Ascii_To_Hex(pSrc, buf, tmp);    // 转换SMSC号码到目标PDU串
    Hex_Num_Decode(buf,(*pSmstype).SCA,tmp);//取短信中心号码,保存，回复时用,是HEX码保存的  
    pSrc += tmp;        // 指针后移,此时到了PDUType
    templen+=tmp;
  
    
    // TPDU段基本参数、回复地址等
     //--PDUType
    Ascii_To_Hex(pSrc, &tmp, 2);    // 取基本参数
    pSrc += 2;        // 指针后移
    templen+=2;
    //--OA----star
        // 包含回复地址，取回复地址信息
    Ascii_To_Hex(pSrc, &tmp, 2);    // 取长度,OA的长度是指具体的号码长度，
    if(tmp & 1) tmp += 1;    // 调整奇偶性
    pSrc += 4;          // 指针后移，除去长度，和91,共4个字节
    templen+=4;
    if(tmp>0&&tmp<=12*2)
    {
    Ascii_To_Hex(pSrc, buf, tmp); 
    Hex_Num_Decode(buf,(*pSmstype).TPA,tmp) ;  // 取TP-RA号码,保存回复地址
    pSrc += tmp;        // 指针后移
    templen+=tmp;
    //--OA---End-------

    ///0891683108200245F320 0D90 683128504568F3 0008 31503241943023 5E 00540057003700300033002300440045005600490043004500490044002800310033003300340035003600370034003800360033002900230049005000310028003100320035002E00330038002E003100380032002E0036003000296D25
    //---SCA
    // TPDU段协议标识、编码方式、用户信息等
    Ascii_To_Hex(pSrc, (u8*)&(*pSmstype).TP_PID, 2);    // 取协议标识(TP-PID)
    pSrc += 2;
    templen+=2;       // 指针后移
    Ascii_To_Hex(pSrc, (u8*)&(*pSmstype).TP_DCS, 2);    // 取编码方式(TP-DCS),这个解码时和回复时用
    pSrc += 2;        // 指针后移
    templen+=2;
    //GsmSerializeNumbers(pSrc, m_SmsType.TP_SCTS, 14);        // 服务时间戳字符串(TP_SCTS) 
    pSrc += 14;       // 指针后移
    templen+=14;
    Ascii_To_Hex(pSrc, &tmp, 2);    // 用户信息长度(TP-UDL)
    pSrc += 2;        // 指针后移
    templen+=2;
   // pDst.TP_DCS=8;
    if(((*pSmstype).TP_DCS&0x0c) == GSM_7BIT)    
    {
        // 7-bit解码
        tmp16=tmp%8 ? ((u16)tmp * 7 / 8 + 1) : ((u16)tmp * 7 / 8);
        tmp16*=2;
       if((templen+tmp16<=pSrcLength)&&(tmp16<400))
       { 
        nDstLength = Ascii_To_Hex(pSrc, buf,tmp16);  // 格式转换
        GsmDecode7bit(buf, DataDst, nDstLength);    // 转换到TP-DU
        nDstLength = tmp;
       }
    }
    else if(((*pSmstype).TP_DCS&0x0c) == GSM_UCS2)
    {
        // UCS2解码
       tmp16=tmp * 2;
        if((templen+tmp16<=pSrcLength)&&(tmp16<400))
        {
        nDstLength = Ascii_To_Hex(pSrc, buf,tmp16);        // 格式转换
        nDstLength = GsmDecodeUcs2(buf, DataDst, nDstLength);    // 转换到TP-DU
        }
    }
    else
    {
        // 8-bit解码
        tmp16=tmp * 2;
       if((templen+tmp16<=pSrcLength)&&(tmp16<512))
        {
        nDstLength = Ascii_To_Hex(pSrc, buf,tmp16);        // 格式转换
        nDstLength = GsmDecode8bit(buf, DataDst, nDstLength);    // 转换到TP-DU
		}
		
    }
   }
   }
    // 返回目标字符串长度
    return nDstLength;
}
 
 //不加短信中心号码
u16   GsmEncodePdu_NoCenter(const SmsType pSrc,const u8 *DataSrc,u16 datalen, u8* pDst)
{
	
	u16 nLength;             // 内部用的串长度
    u16 nDstLength=0;          // 目标PDU串长度
    u8 buf[256];  // 内部用的缓冲区
    
    // SMSC地址信息段
    buf[0]=0;//SCA
     nDstLength += Hex_To_Ascii(buf, pDst, 1);
     // TPDU段基本参数、目标地址等 
    buf[0]=0x11;//PDUTYPE
    buf[1]=0x00;//MR
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 2);
    // SMSC地址信息段
    nLength = Que_Number_Length(pSrc.TPA);// TP-DA地址字符串的长度
    buf[0] = (u8)nLength;   // 目标地址数字个数(TP-DA地址字符串真实长度)
    buf[1] = 0x91;            // 固定: 用国际格式号码
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 2);
    nLength=Hex_Num_Encode(pSrc.TPA,buf,nLength);
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],nLength);  // 转换TP-DA到目标PDU串
    
    // TPDU段协议标识、编码方式、用户信息等
    buf[0] = 0;        // 协议标识(TP-PID)
    buf[1] = pSrc.TP_DCS&0x0c;        // 用户信息编码方式(TP-DCS)
    buf[2] = 0x8f;           // 有效期(TP-VP)为12小时
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],3);
    if((pSrc.TP_DCS&0x0c) == GSM_7BIT)    
    {
        // 7-bit编码方式
        buf[0] = datalen;            // 编码前长度.7位方式表示编码前的长度
        nLength = GsmEncode7bit(DataSrc, &buf[1], datalen); 
        nLength+=1;
		// 转换		TP-DA到目标PDU串
    }
    else if((pSrc.TP_DCS&0x0c)  == GSM_UCS2)
    {
        // UCS2编码方式
        buf[0] = GsmEncodeUcs2(DataSrc, &buf[1], datalen);    // 转换TP-DA到目标PDU串
        nLength = buf[0] + 1;        // nLength等于该段数据长度
    }
    else
    {
        // 8-bit编码方式
        buf[0] = GsmEncode8bit(DataSrc, &buf[1], datalen);    // 转换TP-DA到目标PDU串
        nLength = buf[0] + 1;        // nLength等于该段数据长度
    }
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], nLength);        // 转换该段数据到目标PDU串
    
    // 返回目标字符串长度
    return nDstLength;
}


u16   AnySmsEncode_NoCenter(const u8 *SrcNumber,u8 type,const u8 *DataSrc,u16 datalen, u8* pDst)
{
   SmsType  tmpSms;
   u8 len8;
   len8=Que_Number_Length(SrcNumber);
   if(*SrcNumber==0x86)		///有0x86标记
   	{
	   len8=(len8+1)>>1;
	   //tmpSms.TPA[0]=0x86;
	   memcpy(&tmpSms.TPA[0],SrcNumber,len8);
   	}
   else						///没有0x86标记
   	{
	   len8=(len8+1)>>1;
	   tmpSms.TPA[0]=0x86;
	   memcpy(&tmpSms.TPA[1],SrcNumber,len8);
   	}
   tmpSms.TP_DCS=type&0x0c;
   tmpSms.TP_PID=0;
    return (GsmEncodePdu_NoCenter(tmpSms,DataSrc,datalen,pDst));
}


u16 GsmEncodePdu_Center(const SmsType pSrc,const u8 *DataSrc,u16 datalen, u8* pDst)
{
	
	u16 nLength;             // 内部用的串长度
	u16 nDstLength=0;          // 目标PDU串长度
	u8 buf[256];  // 内部用的缓冲区
    
     // SMSC地址信息段
    nLength = Que_Number_Length(pSrc.SCA);    // SMSC地址字符串的长度    
    buf[0] = (u8)((nLength & 1) == 0 ? nLength : nLength + 1) / 2 + 1;    // SMSC地址信息长度
    buf[1] = 0x91;        // 固定: 用国际格式号码
    nDstLength = Hex_To_Ascii(buf, pDst, 2);        // 转换2个字节到目标PDU串
    nLength=Hex_Num_Encode(pSrc.SCA,buf,nLength);
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],nLength);     // 转换SMSC到目标PDU串
     // TPDU段基本参数、目标地址等
    buf[0]=0x11;//PDUTYPE
    buf[1]=0x00;//MR
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 2);
    // SMSC地址信息段
    nLength = Que_Number_Length(pSrc.TPA);// TP-DA地址字符串的长度
    buf[0] = (u8)nLength;   // 目标地址数字个数(TP-DA地址字符串真实长度)
    buf[1] = 0x91;            // 固定: 用国际格式号码
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 2);
    nLength=Hex_Num_Encode(pSrc.TPA,buf,nLength);
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],nLength);  // 转换TP-DA到目标PDU串
    
    // TPDU段协议标识、编码方式、用户信息等
    buf[0] = 0;        // 协议标识(TP-PID)
    buf[1] = pSrc.TP_DCS&0x0c;        // 用户信息编码方式(TP-DCS)
    buf[2] = 0x8f;            // 有效期(TP-VP)为12小时
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],3);
    if((pSrc.TP_DCS&0x0c) == GSM_7BIT)    
    {
        // 7-bit编码方式
        buf[0] = datalen;            // 编码前长度.7位方式表示编码前的长度
        nLength = GsmEncode7bit(DataSrc, &buf[1], datalen); 
        nLength+=1;
		// 转换		TP-DA到目标PDU串
    }
    else if((pSrc.TP_DCS&0x0c)== GSM_UCS2)
    {
        // UCS2编码方式
        buf[0] = GsmEncodeUcs2(DataSrc, &buf[1], datalen);    // 转换TP-DA到目标PDU串
        nLength = buf[0] + 1;        // nLength等于该段数据长度
    }
    else
    {
        // 8-bit编码方式
        buf[0] = GsmEncode8bit(DataSrc, &buf[1], datalen);    // 转换TP-DA到目标PDU串
        nLength = buf[0] + 1;        // nLength等于该段数据长度
    }
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], nLength);        // 转换该段数据到目标PDU串
    
    // 返回目标字符串长度
    return nDstLength;
}
/*
// 两两颠倒的字符串转换为正常顺序的字符串
// 如："8613693092030" -. "683196032930F0"
// pSrc: 源字符串指针
// pDst: 目标字符串指针
// nSrcLength: 源字符串长度
// 返回: 目标字符串长度
  INT16U  GsmSerializeNumbers(const INT8U* pSrc, INT8U* pDst, INT16U nSrcLength)
{
	
	INT16U nDstLength;   // 目标字符串长度
    INT8U ch;          // 用于保存一个字符
    
    // 复制串长度
    nDstLength = nSrcLength;
	  // 两两颠倒
    for(INT16U i=0; i<nSrcLength;i+=2)
    {
        ch = *pSrc++;        // 保存先出现的字符
        *pDst++ = *pSrc++;   // 复制后出现的字符
        *pDst++ = ch;        // 复制先出现的字符
    }
    
    // 最后的字符是'F'吗？
    if(*(pDst-1) == 'F')
    {
        pDst--;
        nDstLength--;        // 目标字符串长度减1
    }
    
    // 输出字符串加个结束符
    *pDst = '\0';
    
    // 返回目标字符串长度
    return nDstLength;
}

//PDU串中的号码和时间，都是两两颠倒的字符串。利用下面两个函数可进行正反变换：
// 正常顺序的字符串转换为两两颠倒的字符串，若长度为奇数，补'F'凑成偶数
// 如："8613693092030" -. "683196032930F0"
// pSrc: 源字符串指针
// pDst: 目标字符串指针
// nSrcLength: 源字符串长度
// 返回: 目标字符串长度
 INT16U  GsmInvertNumbers(const INT8U* pSrc, INT8U* pDst, INT16U nSrcLength)
{
	
	 INT16U nDstLength;   // 目标字符串长度
    INT8U ch;          // 用于保存一个字符
    
    // 复制串长度
    nDstLength = nSrcLength;
    
    // 两两颠倒
    for(INT16U i=0; i<nSrcLength;i+=2)
    {
        ch = *pSrc++;        // 保存先出现的字符
        *pDst++ = *pSrc++;   // 复制后出现的字符
        *pDst++ = ch;        // 复制先出现的字符
    }
    
    // 源串长度是奇数吗？
    if(nSrcLength & 1)
   {
        *(pDst-2) = 'F';     // 补'F'
        nDstLength++;        // 目标串长度加1
    }
    
    // 输出字符串加个结束符
    *pDst = '\0';
    
    // 返回目标字符串长度
    return nDstLength;
}

//加短信中心号码
  INT16U  GsmEncodePdu(const SM_PARAM* pSrc, INT8U* pDst)
{
	
	INT16U nLength;             // 内部用的串长度
    INT16U nDstLength;          // 目标PDU串长度
    INT8U buf[256];  // 内部用的缓冲区
    
    // SMSC地址信息段
    nLength = strlen(pSrc.SCA);    // SMSC地址字符串的长度    
    buf[0] = (INT8U)((nLength & 1) == 0 ? nLength : nLength + 1) / 2 + 1;    // SMSC地址信息长度
    buf[1] = 0x91;        // 固定: 用国际格式号码
    nDstLength = Hex_To_Ascii(buf, pDst, 2);        // 转换2个字节到目标PDU串
    nDstLength += GsmInvertNumbers(pSrc.SCA, &pDst[nDstLength], nLength);    // 转换SMSC到目标PDU串
     // TPDU段基本参数、目标地址等
    nLength = strlen(pSrc.TPA);    // TP-DA地址字符串的长度
	CString rec_number=CString(pSrc.TPA);
    buf[0] = 0x11;            // 是发送短信(TP-MTI=01)，TP-VP用相对格式(TP-VPF=10)
    buf[1] = 0;               // TP-MR=0
    buf[2] = (INT8U)nLength;   // 目标地址数字个数(TP-DA地址字符串真实长度)
    buf[3] = 0x91;            // 固定: 用国际格式号码
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 4);  // 转换4个字节到目标PDU串
    nDstLength += GsmInvertNumbers(pSrc.TPA, &pDst[nDstLength], nLength); // 转换TP-DA到目标PDU串
    
    // TPDU段协议标识、编码方式、用户信息等
    nLength = strlen(pSrc.TP_UD);    // 用户信息字符串的长度
    buf[0] = pSrc.TP_PID;        // 协议标识(TP-PID)
    buf[1] = pSrc.TP_DCS;        // 用户信息编码方式(TP-DCS)
    buf[2] = 0;            // 有效期(TP-VP)为5分钟
    if(pSrc.TP_DCS == GSM_7BIT)    
    {
        // 7-bit编码方式
        buf[3] = nLength;            // 编码前长度
        nLength = GsmEncode7bit(pSrc.TP_UD, &buf[4], nLength+1) + 4; 
		// 转换		TP-DA到目标PDU串
    }
    else if(pSrc.TP_DCS == GSM_UCS2)
    {
        // UCS2编码方式
        buf[3] = GsmEncodeUcs2(pSrc.TP_UD, &buf[4], nLength);    // 转换TP-DA到目标PDU串
        nLength = buf[3] + 4;        // nLength等于该段数据长度
    }
    else
    {
        // 8-bit编码方式
        buf[3] = GsmEncode8bit(pSrc.TP_UD, &buf[4], nLength);    // 转换TP-DA到目标PDU串
        nLength = buf[3] + 4;        // nLength等于该段数据长度
    }
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], nLength);        // 转换该段数据到目标PDU串
    
    // 返回目标字符串长度
    return nDstLength;
}
*/


