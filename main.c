
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#define ARM_MATH_CM3
#include "arm_math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C
//#define PI 3.1415926535897932384626433832795

uint32_t micros=0;

float gBios[3]={0.0,0.0,0.0};
float aBios[3]= { 496.0 * 4.0 / 32768.0, 244.0 * 4.0 / 32768.0, -343.0 * 4.0 / 32768.0};
float mBios[3]={0.0,0.0,0.0};
float mScale[3]={0.0,0.0,0.0};
int16_t mc[6] = { -195.0, 369.0, -309.0, 257.0, -362.0, 329.0};  			 // Naudojama mBios ir mScale apskaiciavimui;

float k[3] = {0.0, 0.0, 0.0};		// Grazinami kampai is kampu skaiciavimo
float a[3], t, g[3], m[3];					// Grazinama is IMU
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// Grazinama is Madgwick filtro
float z[3]={0.0,0.0,0.0};

uint8_t GpsF=0;
uint8_t ImuF=0;
uint8_t MadqF=0;

// ______________________ Laiko kintamieji Madgwick filtrui _________________________

float deltat = 0.0f, sum = 0.0f;        	// integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        									// used to calculate integration interval

// _______________________________________________________________________________________

double Ilg,Plat,Horp,COG,SOG; 			// Grazinama is GPS
double X,Y,R=6378.137; 			// kintamieji CorToM it MToCor keitimui

// ___________________________ Kintamieji Kalmano filtrui _______________________________

//double DaugMAts[36];	// tarpinis kintamasis matricoms skaiciuoti
double Kx[4][1]={{3},
				{3},
				{0.0},
				{0.0}};		// Kalmano filtruoti kintamieji
double P[4][4]={{1.0,0.0,0.0,0.0},
				{0.0,1.0,0.0,0.0},
				{0.0,0.0,1.0,0.0},
				{0.0,0.0,0.0,1.0}};			// tikimybes
double dt=1;				// laikas skaiciuoti 
double Gx[4][1]={{2},		// Gps kintamuju matrica
				{1},
				{-1}, 
				{1}};//px,py,vx,vy inicializcijos vieta ir greitis 
double u[2][1]={{0},			// pakreitis x
				{0}};			// y

// ______________________________________________________________________________________


struct Blue_paketOUT					// Paketas BlueTooth issiuntimui
{
	uint8_t N;
	int32_t Ilg;
	int32_t Plat;
	int32_t Horp;
	int32_t a;
	int32_t k;
	int32_t KIlg;
	int32_t KPlat;
	
}duomenys;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
double Pavertimas(char kab1,char kab2,uint8_t* mas)  		// Paverimas is ASCII i skaiciu
{
	double ats=0.0;
	
	if ((kab2-kab1)>1)
	{
		double daug=1.0;
		double dal=1.0;
		
		for(int i=kab2-1;i>kab1;i--)
		{
			if(mas[i]!='.')
			{
				ats+=(double)(mas[i]-48)*daug;
				daug*=10.0;
			}else
			{
				dal=daug;
			}
		}
		ats/=dal;
	}
	return (double)ats;
}
void GetGPS() 			// Gaunami duomenys is GPS
{
	uint8_t data;
	
	HAL_UART_Receive(&huart2, &data, 1,2);
	if (data=='$')			// Jeigu simbolis atitinka rasome y GPS raw
	{

		//GpsF=1;
		uint8_t GPSraw[200];
		uint8_t Kab[20];
		double PSek,PLaip,PSek1,ISek,ILaip,ISek1;
		int i=0;  // buferiui
		while(data!=0x0A)			// Surasome i GPSraw
		{
			HAL_UART_Receive(&huart2, &data, 1, 2);
			GPSraw[i]=data;
			i++;
		}
		
		int j=0;
		for(int o=0;o<i;o++) 			//sudarome kableliu masyva
		{
			if (GPSraw[o]==',')
			{
				Kab[j]=o; 
				j++;
			}
		}
		data=0;
		if (GPSraw[2]=='R' & GPSraw[3]=='M' & GPSraw[4]=='C')				//Nuskaitome paketa kurio pavadinimas GNRMC ir isvedame platumos ir ilgumos koordinates
		{
			//Platumos skaiciavimo pradzia
			PSek=Pavertimas(Kab[2]+2,Kab[3],GPSraw);
			PLaip=Pavertimas(Kab[2],(Kab[2]+3),GPSraw);
			PSek1=PSek/60.0;
			Plat=PLaip+PSek1;
			//platumos skaiciavimo pabaiga
			//Ilgumos skaiciavimo pradzia
			ISek=Pavertimas(Kab[4]+3,Kab[5],GPSraw);
			ILaip=Pavertimas(Kab[4],(Kab[4]+4),GPSraw);
			ISek1=ISek/60.0;
			Ilg=ILaip+ISek1;
			//Ilgumos skaiciavimo pabaiga
			COG=Pavertimas(Kab[7],Kab[8],GPSraw);			//azimutas 
		}
		
		if (GPSraw[2]=='G'& GPSraw[3]=='G'& GPSraw[4]=='A')			// GGA
		{
			//Platumos skaiciavimo pradzia
			PSek=Pavertimas(Kab[1]+2,Kab[2],GPSraw);
			PLaip=Pavertimas(Kab[1],(Kab[1]+3),GPSraw);
			PSek1=PSek/60.0;
			Plat=PLaip+PSek1;
			//platumos skaiciavimo pabaiga
			//Ilgumos skaiciavimo pradzia
			ISek=Pavertimas(Kab[3]+3,Kab[4],GPSraw);
			ILaip=Pavertimas(Kab[3],(Kab[3]+4),GPSraw);
			ISek1=ISek/60.0;
			Ilg=ILaip+ISek1;
			//Ilgumos skaiciavimo pabaiga		
			//Paklaidos skaiciavimas	
			Horp=Pavertimas(Kab[7],Kab[8],GPSraw);//horizontali paklaida
			//skaiciavimo pabaiga
		}
		if (GPSraw[2]=='G'& GPSraw[3]=='L'& GPSraw[4]=='L')			// GLL
		{
			//Platumos skaiciavimo pradzia
			PSek=Pavertimas(Kab[0]+2,Kab[1],GPSraw);
			PLaip=Pavertimas(Kab[0],(Kab[0]+3),GPSraw);
			PSek1=PSek/60.0;
			Plat=PLaip+PSek1;
			//platumos skaiciavimo pabaiga
			//Ilgumos skaiciavimo pradzia
			ISek=Pavertimas(Kab[2]+3,Kab[3],GPSraw);
			ILaip=Pavertimas(Kab[2],(Kab[2]+4),GPSraw);
			ISek1=ISek/60.0;
			Ilg=ILaip+ISek1;
			//Ilgumos skaiciavimo pabaiga				
		}
		if (GPSraw[2]=='V' & GPSraw[3]=='T' & GPSraw[4]=='G')		// VTG
		{
			
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);	
			COG=Pavertimas(Kab[0],Kab[1],GPSraw);			//azimutas 
			SOG=Pavertimas(Kab[6],Kab[7],GPSraw);			//greitis 
			//skaiciavimo pabaiga
		}
	}
}
void BlueOut()		// Issiuciamas BlueTooth paketas
{
	duomenys.N='P';
	duomenys.a=(int32_t)(z[0]*10000000);
	duomenys.k=(int32_t)(k[2]*10000000);
	HAL_UART_Transmit(&huart2, &duomenys.N,32,100);
}
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) 
{
    // Set register address
	//HAL_I2C_Master_Transmit(&hi2c1, Address<<1, &Register, 1, 10);
	
    // Read Nbytes
	//HAL_I2C_Master_Receive(&hi2c1, (Address<<1)+1, Data, Nbytes, 100);
	//HAL_I2C_Master_Receive_DMA(&hi2c1, (Address<<1)+1,  Data, Nbytes);
	HAL_I2C_Mem_Read(&hi2c1, (Address<<1)+1, Register, 1, Data, Nbytes, 100);

}
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
	//uint8_t buf[2]={Register,Data};
    // Set register address
	//HAL_I2C_Master_Transmit(&hi2c1, Address<<1, buf, 2, 10);
	HAL_I2C_Mem_Write(&hi2c1, Address<<1, Register,1, &Data, 1, 10);
	
}
void CalCK(uint8_t* Mas, uint8_t S) 		// Gps komandai CRC skaiciuoti
{     
  uint8_t CK_A = 0, CK_B = 0;
  for (int i = 2; i < (S - 2); i++) {
    CK_A = CK_A + Mas[i];
    CK_B = CK_B + CK_A;
  }
  Mas[S - 2] = CK_A;
  Mas[S - 1] = CK_B;
}
void GPS_Init()			// GPS Inicializacija
{
  uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
  HAL_UART_Transmit(&huart1,Disable_GPGSV, 11,1000);
  HAL_Delay(350);
	
  uint8_t Disable_GPGSA[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFD, 0x15};
  CalCK(Disable_GPGSA, 11);
  HAL_UART_Transmit(&huart1,Disable_GPGSA, 11,1000);
  HAL_Delay(350);
	
//  uint8_t Enable_GPGSBS[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x09, 0x01, 0xFD, 0x15};
//  CalCK(Enable_GPGSBS, 11);
//  HAL_UART_Transmit(&huart1,Enable_GPGSBS, 11,1000);
//  HAL_Delay(350);
}
void IMU_Init()				// IMU Inicializacija
{
	#define GYRO_FULL_SCALE_250_DPS 0x00
	#define GYRO_FULL_SCALE_500_DPS 0x08
	#define GYRO_FULL_SCALE_1000_DPS 0x10
	#define GYRO_FULL_SCALE_2000_DPS 0x18

	#define ACC_FULL_SCALE_2_G 0x00
	#define ACC_FULL_SCALE_4_G 0x08
	#define ACC_FULL_SCALE_8_G 0x10
	#define ACC_FULL_SCALE_16_G 0x18
	
  I2CwriteByte(MPU9250_ADDRESS, 0x6B, 0x80); // set restart mode bit (8)
  HAL_Delay(500); // Delay
  I2CwriteByte(MPU9250_ADDRESS, 0x6B, 0x01);
	HAL_Delay(200);
  I2CwriteByte(MPU9250_ADDRESS, 0x6C, 0x00);
  HAL_Delay(200);
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);


  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x00);
  HAL_Delay(20);
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x0F);
  HAL_Delay(20);
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x00);
  HAL_Delay(20);
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);
  HAL_Delay(20);
}
void GyroCalibration()
{
	int32_t Sum[3] = {0, 0, 0};
  for (int i = 0; i < 1000; i++) 
  {
    uint8_t Buf[6];
    I2Cread(MPU9250_ADDRESS, 0x43, 6, Buf);

    Sum[0] += (int16_t)Buf[0] << 8 | (int16_t)Buf[1];    //g
    Sum[1] += (int16_t)Buf[2] << 8 | (int16_t)Buf[3];
    Sum[2] += (int16_t)Buf[4] << 8 | (int16_t)Buf[5];
    HAL_Delay(1);
  }
  gBios[0] = (float)Sum[0] / 1000.0 * 2000.0 / 32768.0;
  gBios[1] = (float)Sum[1] / 1000.0 * 2000.0 / 32768.0;
  gBios[2] = (float)Sum[2] / 1000.0 * 2000.0 / 32768.0;

}
void GetIMU()
{
		// ::: accelerometer and gyroscope :::
	uint8_t INT;
	I2Cread(MPU9250_ADDRESS, 0x3A,1,&INT);
	if ( INT & 0x01)
	{
		
		ImuF=1;
		// Read accelerometer and gyroscope
		uint8_t Buf[14];
		I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

		// Create 16 bits values from 8 bits data

		// Accelerometer
		int16_t ax = (int16_t)Buf[0] << 8 | (int16_t)Buf[1];
		int16_t ay = (int16_t)Buf[2] << 8 | (int16_t)Buf[3];
		int16_t az = (int16_t)Buf[4] << 8 | (int16_t)Buf[5];

		a[0] = (float)ax * 4.0 / 32768.0;
		a[1] = (float)ay * 4.0 / 32768.0;
		a[2] = (float)az * 4.0 / 32768.0;

		a[0] -= aBios[0];
		a[1] -= aBios[1];
		a[2] -= aBios[2];

		// Temperature
		
		int16_t temperature = (int16_t)Buf[6] << 8 | (int16_t)Buf[7];
		t=(float)temperature / 340.0 + 21.0;//36.53;

		// Gyroscope
		int16_t gx = (int16_t)Buf[8] << 8 | (int16_t)Buf[9];
		int16_t gy = (int16_t)Buf[10] << 8 | (int16_t)Buf[11];
		int16_t gz = (int16_t)Buf[12] << 8 | (int16_t)Buf[13];

		g[0] = (float)gx * 2000.0 / 32768.0;
		g[1] = (float)gy * 2000.0 / 32768.0;
		g[2] = (float)gz * 2000.0 / 32768.0;

		g[0] -= gBios[0];
		g[1] -= gBios[1];
		g[2] -= gBios[2];
	}
   // ::: Magnetometer :::


  // Read register Status 1 and wait for the DRDY: Data Ready

  uint8_t ST1;
  I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  if (ST1 & 0x01)
	{
		int16_t mx, my, mz;
		// Read magnetometer data
		uint8_t Mag[7];
		I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

		// Create 16 bits values from 8 bits data

		// Magnetometer
		if (!(Mag[6] & 0x08)) {
			mx = (int32_t)(((int16_t)Mag[1]) << 8 | (int16_t)Mag[0]);
			my = (int32_t)(((int16_t)Mag[3]) << 8 | (int16_t)Mag[2]);
			mz = (int32_t)(((int16_t)Mag[5]) << 8 | (int16_t)Mag[4]);
		}

		if (mx < mc[0])mc[0] = mx;
		if (mx > mc[1])mc[1] = mx;
		if (my < mc[2])mc[2] = my;
		if (my > mc[3])mc[3] = my;
		if (mz < mc[4])mc[4] = mz;
		if (mz > mc[5])mc[5] = mz;

		mBios[0] = (mc[0] + mc[1]) / 2.0;
		mBios[1] = (mc[2] + mc[3]) / 2.0;
		mBios[2] = (mc[4] + mc[5]) / 2.0;

		float mcs[4];
		mcs[0] = (mc[1] - mc[0]) / 2.0;
		mcs[1] = (mc[3] - mc[2]) / 2.0;
		mcs[2] = (mc[5] - mc[4]) / 2.0;
		mcs[3] = (mcs[0] + mcs[1] + mcs[2]) / 3.0;
		mScale[0] = mcs[0] / mcs[3];
		mScale[1] = mcs[1] / mcs[3];
		mScale[2] = mcs[2] / mcs[3];

		mx -= mBios[0];
		my -= mBios[1];
		mz -= mBios[2];

		m[0] = (float)mx * 10.0 * 4912.0 / 32768.0 * mScale[0];
		m[1] = (float)my * 10.0 * 4912.0 / 32768.0 * mScale[1];
		m[2] = (float)mz * 10.0 * 4912.0 / 32768.0 * mScale[2];
	}
}
void MadgwickAHRS(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q)
{
	float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
	float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
	float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	// updateParams()
        float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
        float norm;
        float hx, hy, _2bx, _2bz;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q1mx;
        float _2q1my;
        float _2q1mz;
        float _2q2mx;
        float _4bx;
        float _4bz;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _2q4 = 2.0f * q4;
        float _2q1q3 = 2.0f * q1 * q3;
        float _2q3q4 = 2.0f * q3 * q4;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;
        gx *= PI / 180.f;
        gy *= PI / 180.f;
        gz *= PI / 180.f;

        // updateTime()
        Now = HAL_GetTick();
        deltat = ((Now - lastUpdate) / 1000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = Now;

        // Normalise accelerometer measurement
        norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrtf(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2.0f * q1 * mx;
        _2q1my = 2.0f * q1 * my;
        _2q1mz = 2.0f * q1 * mz;
        _2q2mx = 2.0f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        norm = 1.0f/norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * deltat;
        q2 += qDot2 * deltat;
        q3 += qDot3 * deltat;
        q4 += qDot4 * deltat;
        norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        norm = 1.0f/norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;
}
void Kampu_Skaiciavimas()
{
  float a12 =   2.0f * (q[1] * q[2] - q[0] * q[3]);
  float a22 =   2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f;
  float a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
  float a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
  float a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
/*
  k[2] = atan2(a12, a22);   // psi
  k[0] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]);                      // theta
  k[1] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
*/
  float pitch = atan(a31 / sqrt(a32 * a32 + a33 * a33));
  float roll  = atan(a32 / sqrt(a31 * a31 + a33 * a33));
  float yaw   = atan2(a12, a22);
  pitch *= 180.0f / PI;
  roll  *= 180.0f / PI;
  yaw   *= 180.0f / PI;
  float magnetic_declination = 7.15; // Kaunas
  //yaw   += magnetic_declination;
  //  if      (yaw >= +180.f) yaw -= 360.f;
  //  else if (yaw <  -180.f) yaw += 360.f;
  k[0] = pitch;
  k[1] = roll;
  k[2] = -yaw;
}
void quaternProd(float* a, float* b, float* ab) 
{
  ab[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  ab[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  ab[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  ab[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}
void quaternConj(float* q, float* qConj) 
{
  qConj[0] = q[0];
  qConj[1] = -q[1];
  qConj[2] = -q[2];
  qConj[3] = -q[3];
}
void quaternRotate(float* v, float* q) 
{
  float vv[] = {0, v[0], v[1], v[2], v[3]};
  float v0XYZ[] = {0, 0, 0, 0};
  float v0XYZv1[] = {0, 0, 0, 0};
  float qConj[] = {0, 0, 0, 0};
  //quaternProd(quaternProd(q, {0, v}), quaternConj(q));
  quaternProd(q, vv, v0XYZ);
  quaternConj(q, qConj);
  quaternProd(v0XYZ, qConj, v0XYZv1);
  v[0] = v0XYZv1[1];
  v[1] = v0XYZv1[2];
  v[2] = v0XYZv1[3];
}
void lean(float* in, float* angle, float* out) 
{
  float nv[3] = {0, 0, 0};
  float nvv[3] = {0, 0, 0};
  //float out[]={0,0,0};
  for (int i = 0; i < 3; i++) {
    out[i] = in[i];
  }
  for (int i = 0; i < 2; i++) {
    nvv[i] = 1;
    nv[0] = nvv[1] * out[2] - nvv[2] * out[1];
    nv[1] = nvv[0] * out[2] - nvv[2] * out[0];
    nv[2] = nvv[0] * out[1] - nvv[1] * out[0];
    nvv[i] = 0;
    
    for (int j = 0; j < 2; j++) {
      out[j] = cos(angle[i] / 180.0 * PI) * out[j] + sin(angle[i] / 180.0 * PI) * nv[j];
    }
  }
}
void CorToM()
{
	double dlat=Plat*PI/180.0;
	double A=sin(dlat/2.0)*sin(dlat/2.0);
	double C=2.0*atan2(sqrt(A),sqrt(1.0-A));
	Y=R*C;
	double dlon=Ilg*PI/180.0;
	A=cos(Plat*PI/180.0)*cos(Plat*PI/180.0)*sin(dlon/2.0)*sin(dlon/2.0);
	C=2*atan2(sqrt(A),sqrt(1.0-A));
	X=R*C;
}
void MToCor()
{
	double C=Y/R;
	double A=1.0/(1.0+(1.0/(tan(C/2.0)*tan(C/2.0))));
	double dlat=2.0*asin(sqrt(A));
	Plat=dlat*180.0/PI;
	C=X/R;
	A=1.0/(1.0+(1.0/(tan(C/2.0)*tan(C/2.0))));
	double dlon=2.0*asin(sqrt(A/(cos(Plat*PI/180.0)*cos(Plat*PI/180.0))));
	Ilg=dlon*180.0/PI;
}

void  ex02_f(double* x, double* u, double* n,/* ats-> */ double* XO_x,double* XO_u,double* XO_n)//, double dt)				//[xo, XO_x, XO_u, XO_n] = 
{
	double px = x[0];
	double py = x[1];
	double vx = x[2];
	double vy = x[3];
	double ax = u[0];
	double ay = u[1];
	double nx = n[0];
	double ny = n[1];

	px = px + vx*dt;
	py = py + vy*dt;
	vx = vx + ax*dt + nx;
	vy = vy + ay*dt + ny;

	x[0] = px;
	x[1] = py;
	x[2] = vx;
	x[3] = vy;
	
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			XO_x[i*4+j]=0;
			if(i==j)
			{
				XO_x[i*4+j]=1;
			}
		}
	}
	XO_x[2]=dt;
	XO_x[7]=dt;
	
	for(int i=0;i<8;i++)
	{
		XO_u[i]=0;
	}
	XO_u[4]=dt;
	XO_u[7]=dt;

	for(int i=0;i<8;i++)
	{
		XO_n[i]=0;
	}
	XO_n[4]=1;
	XO_n[7]=1;

}
void ex02_h(double* x,/* ats-> */double* Y, double* Y_x)        //function [y, Y_x] = 
{					

	double px = x[0];
	double py = x[1];

	double d = sqrt(px*px + py*py);
	double a = atan2(py, px);
	// a = atan(py/ px); % use this only for symbolic Jacobian computation

	Y[0] = d;
	Y[1] = a;
	
	Y_x[0]= px/sqrt(px*px + py*py);
	Y_x[1]= py/sqrt(px*px+ py*py);
	Y_x[2]= 0;
	Y_x[3]= 0;
	Y_x[4]= -py/(px*px*(py*py/px/px + 1));
	Y_x[5]= 1/(px*(py*py/px/px + 1));
	Y_x[6]= 0;
	Y_x[7]= 0;
}
void SumM(double* A,double* B,uint8_t I,uint8_t J)
{
	for(int i=0;i<I;i++)
	{
		for(int j=0;j<J;j++)
		{
			A[i*J+j]+=B[i*J+j];
		}
	}
}
void DaugM(double*A,double* B,uint8_t I,uint8_t J,uint8_t K,uint8_t T,double* C)
{
	double DaugMAts[36];
	for(int i=0;i<I;i++)
	{
		for(int k=0;k<K;k++)
		{
			DaugMAts[i*K+k]=0.0;
			if(T)
			{
				//K=I;
				for(int j=0;j<J;j++)
				{
					DaugMAts[i*K+k]+=A[i*J+j]*B[k*J+j];
				}
			}
			else
			{
				for(int j=0;j<J;j++)
				{
					DaugMAts[i*K+j]+=A[i*J+j]*B[j*K+k];
				}
			}
		}
	}
	for(int i=0;i<I*K;i++)
	{
		C[i]=DaugMAts[i];
	}
}
void SkirtM(double* A,double* B,uint8_t I,uint8_t J)
{
	for(int i=0;i<I;i++)
	{
		for(int j=0;j<J;j++)
		{
			A[i*J+j]-=B[i*J+j];
		}
	}
}
void M_1(double* A)
{
	double det=A[0]*A[3]-A[1]*A[2];
	double temp=A[0];
	A[0]=A[3]/det;
	A[1]*=-1;
	A[1]/=det;
	A[2]*=-1;
	A[2]/=det;
	A[3]=temp/det;
}
void KalmanFilter()
{
	 // simulate
	
//		double q[1][2]={0.050,0.050};
//		double n[1][2]={0.0,0.0};
//		n[0][0]=q[0][0]*((double)rand())/(double)RAND_MAX;
//		n[0][1]=q[0][1]*((double)rand())/(double)RAND_MAX;
//		
		double XO_x[4][4]={{1, 0, dt, 0},
						 {0, 1, 0, dt},
						 {0, 0, 1, 0},
						 {0, 0, 0, 1}};
		double XO_u[4][2]= {{0, 0},
							{0, 0},
							{dt, 0},
							{0, dt}};
		double XO_n[4][2]= {{0, 0},
							{0 ,0},
							{1, 0},
							{0, 1}};
//		ex02_f((double*)Gx, (double*)u, (double*)n,/* ats-> */ (double*) XO_x,(double*) XO_u,(double*) XO_n);
//												
//		double r[1][2]={0.050,0.00872664};
//		double v[1][2]={0.0,0.0};
//		v[0][0]=r[0][0]*((double)rand())/(double)RAND_MAX;
//		v[0][1]=r[0][1]*((double)rand())/(double)RAND_MAX;
//		
		double Ky[2][1];
		double Y_x[2][4] = {{0, 0, 0, 0},
							{0, 0, 0, 0}};
		ex02_h((double*)Gx,/*ats->*/(double*)Ky,(double*)Y_x);
//		SumM((double*)Ky,(double*)v,1,2);
		
		// estimate - prediction
		double zeros[2]={0,0};
    ex02_f((double*) Kx, zeros, zeros,/* ats-> */ (double*) XO_x,(double*) XO_u,(double*) XO_n);
		
		double DaugAts[36];
		for(int i=0;i<36;i++)
		{
			DaugAts[i]=0.0;
		}
		
		DaugM((double*)XO_x,(double*)P,4,4,4,0,DaugAts);
		DaugM((double*)DaugAts,(double*)XO_x,4,4,4,1,(double*)P);
//		for(int i=0;i<4*4;i++)
//		{
//			*(&P[0][0]+i)=DaugAts[i];
//		}
		double Q[2][2]={{0.010, 0.0},
						{0.0, 0.010}};
		DaugM((double*)XO_n,(double*)Q,4,2,2,0,DaugAts);
		DaugM((double*)DaugAts,(double*)XO_n,4,2,4,1,DaugAts);

		SumM((double*)P,DaugAts,4,4);
		
		// correction
		double e[2][1];
		double H[2][4] = {{0, 0, 0, 0},
						{0, 0, 0, 0}};
		ex02_h((double*)Kx,/*ats->*/(double*)e,(double*)H);
    
		DaugM((double*)H,(double*)P,2,4,4,0,DaugAts);
		double E[2][2];
		DaugM((double*)DaugAts,(double*)H,2,4,2,1,(double*)E);
		
//		for(int i=0;i<2*2;i++)
//		{
//			*(&E[0][0]+i)=DaugAts[i];
//		}
		
		double z[2][1];
    z[0][0] = Ky[0][0] - e[0][0];
		z[1][0] = Ky[1][0] - e[1][0];
		
		double Z[2][2];
		double Rk[2][2]={{0.010, 0.0},
						{0.0, PI*PI/180.0/180.0}};
    Z[0][0] = Rk[0][0] + E[0][0];
		Z[0][1] = Rk[0][1] + E[0][1];
		Z[1][0] = Rk[1][0] + E[1][0];
		Z[1][1] = Rk[1][1] + E[1][1];
    
		DaugM((double*)P,(double*)H,4,4,2,1,DaugAts);
		M_1((double*)Z);
		double K[4][2];
		DaugM((double*)DaugAts,(double*)Z,4,2,2,0,(double*)K);
		
//		for(int i=0;i<4*2;i++)
//		{
//			*(&K[0][0]+i)=DaugAts[i];
//		}
		
    DaugM((double*)K,(double*)z,4,2,1,0,DaugAts);
		SumM((double*)Kx,(double*)DaugAts,4,1);
    
		DaugM((double*)K,(double*)H,4,2,4,0,DaugAts);
		DaugM((double*)DaugAts,(double*)P,4,4,4,0,DaugAts);
		SkirtM((double*)P,(double*)DaugAts,4,4);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	/*HAL_MspInit();
	HAL_TIM_OC_MspInit(&htim2);
	TIM2->CR1|=0x1;
	TIM2->DIER|=0x1;*/
	GPS_Init();
	IMU_Init();
	//GyroCalibration();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		GetGPS();
		uint8_t patikra='5';
		HAL_UART_Transmit(&huart3,&patikra, 1, 10);
		//HAL_UART_Receive_DMA(&huart1, GPSraw, 20);
		GetIMU();
		if(ImuF)
		{
			ImuF=0;
			MadgwickAHRS(-a[0], -a[1], a[2], -g[0], -g[1], g[2], m[1], m[0], m[2], q);
			Kampu_Skaiciavimas();
			/*
			float qC[4];
			quaternConj(q, qC);
			quaternRotate(a, qC);
			*/
			lean(a, k, z);
			
		}
		
		if(GpsF)
		{
			duomenys.Ilg=(int32_t)(Ilg*10000000);
			duomenys.Plat=(int32_t)(Plat*10000000);
			duomenys.Horp=(int32_t)(Horp*10000000);
			GpsF=0;
			CorToM();
			
			Gx[2][0]=Gx[0][0]-(X*1000.0);
			Gx[3][0]=Gx[1][0]-(Y*1000.0);
		
			Gx[0][0]=X*1000.0;
			Gx[1][0]=Y*1000.0;
//			SOG*=1000.0/60.0/60.0;
//			COG*=PI/180.0;
//			Gx[2][0]=SOG*sin(COG);
//			Gx[3][0]=SOG*cos(COG);
			//KalmanFilter();
			X=Kx[0][0]/1000.0;
			Y=Kx[1][0]/1000.0;
			MToCor();
			duomenys.KIlg=(int32_t)(Ilg*10000000);
			duomenys.KPlat=(int32_t)(Plat*10000000);
			BlueOut();
		}
			
	
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
