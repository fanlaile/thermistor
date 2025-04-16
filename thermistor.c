
static double Temp_Rref = 200;  //串联的电阻阻值KΩ
static double Temp_R0 = 3643;//95.8188;    //热敏电阻25度时的阻值KΩ
static double Temp_B = 5300;//4300;    //热敏电阻的B值

static uint8_t First_temp = 0, Temp_plan = 0;
static double Temp_NEW_R0 = 95.8188;    //热敏电阻25度时的阻值KΩ
static double Temp_NEW_B = 4300;    //热敏电阻的B值

static int ADC_Value = 0;
static int ADC_Value_Last = 0;
static uint16_t ADC_Error = 0;
static uint16_t ADC_T = 0;

double ADCTemp_Vlaue_Get(void)
{
    do{
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc,10);
        ADC_Value = HAL_ADC_GetValue(&hadc);   //取ADC的值
        HAL_ADC_Stop(&hadc);
        
        ADC_Error = (ADC_Value > ADC_Value_Last) ? (ADC_Value - ADC_Value_Last):(ADC_Value_Last - ADC_Value);
        ADC_T ++;
    }while(ADC_Error > 1000 && ADC_T < 100);    //判断本次测量值和上次测量值偏差过大就重测
    ADC_Value_Last = ADC_Value;
    
    if(ADC_Value > 4090)    //ADC值大于4090代表测温电阻未插入
    {
//        sprintf(cmd, "NTC_Connect_Error!\n");
//        sendToUart1((uint8_t *)cmd);
        return -1000;
    }
    double ADC_Value_V = ADC_Value * 3.3/4096;        //计算电压值
    double Temp_R = ADC_Value_V * Temp_Rref / (3.3 - ADC_Value_V);  //计算热敏电阻阻值
    double Temp_Value = 1 / (1 / 298.15 + (log(Temp_R / Temp_R0)) / Temp_B) - 273.15;   //计算热敏电阻温度值
	
	if(First_temp == 0)//上电第一次读取温度使用就参数
	{
		if(Temp_Value>80)//若大于80，认为是新热敏，则后期全部使用新参数读取温度
		{
			Temp_plan = 1;//
		}
		First_temp = 1;
	}
	if(Temp_plan)//新参数
	{
		Temp_Value = 1 / (1 / 298.15 + (log(Temp_R / Temp_NEW_R0)) / Temp_NEW_B) - 273.15;   //计算热敏电阻温度值
	}
	else//旧参数
	{
		Temp_Value = 1 / (1 / 298.15 + (log(Temp_R / Temp_R0)) / Temp_B) - 273.15;
	}
	
    
//    sprintf(cmd, "ADC:%d  V:%.2f  R:%.2f  Temp:%.2f  ", ADC_Value,ADC_Value_V,Temp_R,Temp_Value);
//    sendToUart1((uint8_t *)cmd);
//    HAL_Delay(5);
    
    return Temp_Value;
}

/*         黄色薄加热贴的参数         */
static double previous_error;   //上一次偏差
static double error = 0;
static double integral = 0.0;   //积分值
static double derivative = 0;
uint8_t Duty_Set = 0;           //占空比
double measured_temp = 0;

static double kp = 5.0;         //比例系数
static double ki = 0.3;         //积分系数
static double kd = 20.0;        //微分系数
static double dt = 1;           //时间1s一次

static uint8_t Duty_Min = 2;    //温控最小占空比
static uint8_t Duty_Max = 100;  //温控最大占空比

static double PID_Down = 20;    //低于目标温度多少度 下偏差 开始PID调节
static double PID_Up = 1;       //高于目标温度多少度 上偏差 关闭PID调节

uint8_t PID_tims = 0;

void Heat_PID(void)
{
	//sendToUart1((uint8_t *)"heat PID\n");
    double Target_Temp = firebar_temp;  //目标温度
    measured_temp = ADCTemp_Vlaue_Get();//温度测量值
    
    if(measured_temp > -10 && measured_temp < 210)  //测量温度值在范围内就开启温控 否则就关闭加热
    {
        PID_temperrflag = 0;
        if(measured_temp > Target_Temp - PID_Down && Target_Temp > 0)  //测量值大于目标温度下偏差 就启动PID
        {
            error = Target_Temp - measured_temp; //计算偏差 目标温度减去测量值
            if(error > 0)   {integral += error * dt;}   //仅在温度低于目标时累积积分
            else if(error < -0.6){integral = 0;}        //温度超过目标温度0.4度 就清空误差积累积分
            derivative = (error - previous_error) / dt;
            double output = kp * error + ki * integral + kd * derivative;
            previous_error = error;
            
            Duty_Set = (uint8_t)output;
            
            if(Duty_Set > Duty_Max){Duty_Set = Duty_Max;}
            if(Duty_Set<Duty_Min || Target_Temp < measured_temp){Duty_Set = Duty_Min;} //如果占空比小于最小值 或者温度大于目标值 占空比设置到最小
            if(error < -PID_Up){Duty_Set = 0;}  //如果温度超过目标上偏差 就关闭加热 保护作用
        }
        else if(measured_temp >120 && Target_Temp > 0)
        {
            uint8_t OpenHeat = 2;   //开启加热2秒
            uint8_t CloseHeat = 2;  //关闭加热4秒
            if(Duty_Set == Duty_Max && PID_tims >= OpenHeat)
            {
                Duty_Set = 0;
                PID_tims = 0;
            }
            else if(Duty_Set == 0 && PID_tims >= CloseHeat)
            {
                Duty_Set = Duty_Max;
                PID_tims = 0;
            }
            PID_tims ++;
        }
        else if(Target_Temp > 0)
        {
            Duty_Set = Duty_Max;
        }
        else
        {
            Duty_Set = 0;
        }
    }
    else
    {
        PID_temperrflag = 1;
        Duty_Set = 0;
        if(PID_errort == 0)
        {
            sendToUart1((const uint8_t *)"M105_meas_error\n");
            PID_errort ++;
        }
    }
    
    HeatSetPwm(Duty_Set);
    

}