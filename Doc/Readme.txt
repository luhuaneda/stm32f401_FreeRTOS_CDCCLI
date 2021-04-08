
初学FreeRTOS时，每次打开函数列表都一脸懵逼？根本不知道这什么prv,v,ux是什么意思，虽然平时使用也不需要知道这么多东西，因为它不怎么影响开发，但是理解总比疑惑好，我们还是把它理清楚一下跟好。

命名规则
u :代表unsigned。

s :代表short。

c :char。
所以类似uc，us类的变量就是unsigned char，unsigned short，分别对应uint8_t,uint16_t。

x :为用户自定义的数据类型，比如结构体，队列等。
常看到ux开头的函数，就是unsigned且用户自定义的类型。需要注意的是size_t变量前缀也是ux。

e :枚举变量

p :指针变量
类似(uint16_t *)变量前缀为pus。

prv :static函数

v: void函数
————————————————
版权声明：本文为CSDN博主「薯条可乐」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/qq_35675731/article/details/89814807


2021.04.06
1.
STM32CUBEMX新建cdc.复制四个文件到项目
FreeRTOS_CLI.c
Sample-CLI-commands.c
serial.c
UARTCommandConsole.c


2.设备管理器更新后，再打开串口助手

3.添加
3.1
E:\Michael\usb2Bus\stmUSB2Bus\Software\stm32f401_FreeRTOS_CDCCLI\Core\Inc\FreeRTOSConfig.h
/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE 1024
/* USER CODE END Defines */

3.2根据
https://github.com/flyoob/NanoVNA-F/blob/009e5608b6ac807862fcc41a21d9496df3e30871/Src/main.c
修改
...

https://github.com/flyoob/NanoVNA-F/blob/009e5608b6ac807862fcc41a21d9496df3e30871/Src/main.c

改为
#define APP_RX_DATA_SIZE  512
#define APP_TX_DATA_SIZE  512

3.1 usbd_cdc_if.h
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  // if (hcdc->TxState != 0){
    // return USBD_BUSY;
  // }
  while (hcdc->TxState != 0);  //  can not always wait
  // USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  // result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  if((Len % 64) == 0)
  {
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    while (hcdc->TxState != 0);
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, 0);
    result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  }
  else
  {
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    while (hcdc->TxState != 0);
  }
  /* USER CODE END 7 */
  return result;
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  BaseType_t      xHigherPriorityTaskWoken = pdFALSE;
    
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    
  vUARTInterruptHandler(Buf, Len);  
    
  return (USBD_OK);
  /* USER CODE END 6 */
}

3.2 serial.c
void vUARTInterruptHandler(uint8_t* Buf, uint32_t *Len)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  uint32_t Cnt = 0;
  while(Cnt < *Len)
  {
    xQueueSendFromISR( xRxedChars, Buf+Cnt, &xHigherPriorityTaskWoken );
    Cnt ++;
  }
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
