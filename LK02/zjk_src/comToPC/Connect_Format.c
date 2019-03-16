#include "Connect_format.h"

//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(uint8_t fun,uint8_t *data,uint8_t len)
{
	uint8_t send_buf[32];
	uint8_t i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
	for(i=0;i<len+4;i++) z_serial_write(&send_buf[i],1);	//�������ݵ�����1 
}
//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	uint8_t tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
}	
//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	uint8_t tbuf[28]; 
	uint8_t i;
	for(i=0;i<28;i++)tbuf[i]=0;//��0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
} 
/*************************************************************
��������λ��,��λ����������λ��V4.2�汾

**************************************************************/
void Data_Send_Status(float Pitch,float Roll,float Yaw,int16_t *gyro,int16_t *accel)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;
	unsigned int _temp;
	uint8_t data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(Roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0-(int)(Pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	//��У��
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//���ڷ�������
	for(i=0;i<_cnt;i++)
		usart1_send_char(&data_to_send[i]);
		
}

//void Send_Pose_Data(int16_t *Gyro,int16_t *Accel)
//{
//	unsigned char i=0;
//	unsigned char _cnt=0,sum = 0;
////	unsigned int _temp;
//	uint8_t data_to_send[50];

//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xF1;
//	data_to_send[_cnt++]=0;
//	

//	data_to_send[_cnt++]=BYTE1(Accel[0]);
//	data_to_send[_cnt++]=BYTE0(Accel[0]);
//	data_to_send[_cnt++]=BYTE1(Accel[1]);
//	data_to_send[_cnt++]=BYTE0(Accel[1]);
//	data_to_send[_cnt++]=BYTE1(Accel[2]);
//	data_to_send[_cnt++]=BYTE0(Accel[2]);
//	
//	data_to_send[_cnt++]=BYTE1(Gyro[0]);
//	data_to_send[_cnt++]=BYTE0(Gyro[0]);
//	data_to_send[_cnt++]=BYTE1(Gyro[1]);
//	data_to_send[_cnt++]=BYTE0(Gyro[1]);
//	data_to_send[_cnt++]=BYTE1(Gyro[2]);
//	data_to_send[_cnt++]=BYTE0(Gyro[2]);
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	
//	data_to_send[3] = _cnt-4;
//	//��У��
//	for(i=0;i<_cnt;i++)
//		sum+= data_to_send[i];
//	data_to_send[_cnt++]=sum;
//	
//	//���ڷ�������
//	for(i=0;i<_cnt;i++)
//		z_serial_write(&data_to_send[i],1);
//}

void Send_Pose_Data(uint16_t *a,uint16_t *b,uint16_t *c)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;
//	unsigned int _temp;
	uint8_t data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;
	

	data_to_send[_cnt++]=BYTE1(a[0]);
	data_to_send[_cnt++]=BYTE0(a[0]);
	data_to_send[_cnt++]=BYTE1(b[0]);
	data_to_send[_cnt++]=BYTE0(b[0]);
	data_to_send[_cnt++]=BYTE1(c[0]);
	data_to_send[_cnt++]=BYTE0(c[0]);
	
	data_to_send[3] = _cnt-4;
	//��У��
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	//���ڷ�������
	for(i=0;i<_cnt;i++)
		z_serial_write(&data_to_send[i],1);
}
void Send_Pose_InData(int16_t *a,uint16_t *b,uint16_t *c)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;
//	unsigned int _temp;
	uint8_t data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;
	

	data_to_send[_cnt++]=BYTE1(a[0]);
	data_to_send[_cnt++]=BYTE0(a[0]);
	data_to_send[_cnt++]=BYTE1(b[0]);
	data_to_send[_cnt++]=BYTE0(b[0]);
	data_to_send[_cnt++]=BYTE1(c[0]);
	data_to_send[_cnt++]=BYTE0(c[0]);
	
	data_to_send[3] = _cnt-4;
	//��У��
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	//���ڷ�������
	for(i=0;i<_cnt;i++)
		z_serial_write(&data_to_send[i],1);
}

void Send_Pose_IData(uint16_t *Gyro,int16_t *Accel)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;
//	unsigned int _temp;
	uint8_t data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;
	

	data_to_send[_cnt++]=BYTE1(Gyro[0]);
	data_to_send[_cnt++]=BYTE0(Gyro[0]);
	data_to_send[_cnt++]=BYTE1(Accel[0]);
	data_to_send[_cnt++]=BYTE0(Accel[0]);
	data_to_send[3] = _cnt-4;
	//��У��
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	//���ڷ�������
	for(i=0;i<_cnt;i++)
		z_serial_write(&data_to_send[i],1);
}