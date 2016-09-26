#include "Calculate.h"
#define AXIS1 1
#define PI 3.1415926

Calculate::Calculate()
{
	Read_Operat= new Read_AD_Dri();
	delete Read_Operat;
	Read_Operat= new Read_AD_Dri();
	Inverse = new Inverse_kinematics();
}

int Calculate::File_Out()
{  
	short sRtn, space;
	float64 AD_Data_Read[6];
	FILE *fp[10];
	double Enc_axis[5];//save encoder number
	//初次记录的编码器数值
	bool teaching = false;// 示教模式开始
	double step = 20.0;//设置步长
	bool Jog_Xp = true, Jog_Xm = true, Jog_Yp = true, Jog_Ym = true;//Flag to determine whether the stick is back at the center position
	bool Jog_Zp = true, Jog_Zm = true, Jog_Rp = true, Jog_Rm = true, Jog_Tp = true, Jog_Tm = true;
	bool Jog_Step = true;// ture: can change the value of step
	//定义一个数组，取50行6列
	double Samp_point[50][6]={0};//最多取点50个
	int num_samp = 1;

	for (int i=0;i<5;i++)
	{
		//读取编码器数值
		sRtn = GT_GetEncPos(AXIS1+i,&Enc_axis[i]);//Enc_axis
	}

	//保存当前数据点，使用joystick
	double ang_jt_S=0,ang_jt_L=0,ang_jt_U=0,ang_jt_R=0,ang_jt_T=0,ang_jt_R_read=0,ang_jt_T_read=0;//当前各关节角度值
	double pre_ang_S=0,pre_ang_L=0,pre_ang_U=0,pre_ang_R=0,pre_ang_T=0;//前步各关节角度值
	bool ref_flag = true;
	double delta_x=0, delta_y =0, delta_z = 0;
	int	time = 0;
	double pos = 0;
	int stop = 0;
	int start = 0;
	int dot = 0;
	int part = 0;
	printf("Please enter the blank button to begin!\n");
	while(1)
	{
		Read_Operat->Read(AD_Data_Read);
		//printf("%f,%f,%f,%f,%f\r",AD_Data_Read[0],AD_Data_Read[1],AD_Data_Read[2],AD_Data_Read[3],AD_Data_Read[4],AD_Data_Read[5]);
		if (kbhit()&&!teaching)
		{
			if(getch()==' '){
				teaching = true;
				printf("already begin!\n");
				printf("s-save q-sampling finish.\n" );
				Sleep(1000);
			}
		}

		if(teaching)//示教取点
		{ 
			if((AD_Data_Read[0]>4)&&Jog_Yp&&!start)//向左，y向+
			{
				delta_y += step;
				Jog_Yp = false;
				start = 1;
			}
			if((AD_Data_Read[0]<1)&&Jog_Ym&&!start)//向右，y向-
			{
				delta_y -= step;
				Jog_Ym =false;
				start = 1;
			}
			if((AD_Data_Read[1]>4)&&Jog_Xp&&!start)//向前，x向+
			{
				delta_x += step;
				Jog_Xp = false;
				start = 1;
			}
			if((AD_Data_Read[1]<1)&&Jog_Xm&&!start)//向后，x向-
			{
				delta_x -= step;
				Jog_Xm = false;
				start = 1;
			}
			if((AD_Data_Read[2]>4)&&Jog_Step&&!start)//正转，step+=5
			{
				step+=5;
				if(step>=50){
					step=50;
				}
				Jog_Step = false;
				printf("step is %f now\n",step);
				start = 1;
			}
			if((AD_Data_Read[2]<1)&&Jog_Step&&!start)//逆转，step-=5
			{
				step-=5;
				if(step<=0){
					step=1;
				}
				Jog_Step = false;
				printf("step is %f now\n",step);
				start = 1;
			}
			if((AD_Data_Read[3]>4)&&Jog_Zp&&!start)//向上，Z向+
			{
				delta_z += step;
				Jog_Zp = false;
				start = 1;
			}
			if((AD_Data_Read[3]<1)&&Jog_Zm&&!start)//向下，z向-
			{
				delta_z -= step;
				Jog_Zm = false;
				start = 1;
			}
			if((AD_Data_Read[0])>2 && AD_Data_Read[0] <3)
			{
				Jog_Yp = true; Jog_Ym = true;
			}
			if((AD_Data_Read[1])>2 && AD_Data_Read[1] <3)
			{
				Jog_Xp = true; Jog_Xm = true;
			}
			if((AD_Data_Read[2])>2 && AD_Data_Read[2] <3)
			{
				Jog_Step = true;
			}
			if((AD_Data_Read[3])>2 && AD_Data_Read[3] <3)
			{
				Jog_Zp = true; Jog_Zm = true;
			}
			if((AD_Data_Read[4]>4)&&Jog_Rp&&!start) {ang_jt_R += step/10; Jog_Rp = false;start = 1;}//腕旋转
			if((AD_Data_Read[4]<1)&&Jog_Rm&&!start) {ang_jt_R -= step/10; Jog_Rm = false;start = 1;}
			if((AD_Data_Read[4]>2)&&(AD_Data_Read[4]<3)){Jog_Rp = true; Jog_Rm = true;}

			if((AD_Data_Read[5]>4)&&Jog_Tp&&!start) {ang_jt_T += step/10; Jog_Tp = false;start = 1;}//指俯仰
			if((AD_Data_Read[5]<1)&&Jog_Tm&&!start) {ang_jt_T -= step/10; Jog_Tm = false;start = 1;}
			if((AD_Data_Read[5]>2)&&(AD_Data_Read[5]<3)){Jog_Tp = true; Jog_Tm = true;}
			//printf("del_x=%f,del_y=%f,del_z=%f\r",delta_x,delta_y,delta_z);
			Inverse->Inverse_calculate(0 + delta_x,790 + delta_y,700+delta_z);//逆向计算各关节角度1,2,3

			//Inverse->Inverse_calculate(0,790,700);
			//设置Axis轴的目标位置,当前角度值
			for (int i=0;i<5;i++)
			{
				//读取编码器数值
				sRtn = GT_GetEncPos(AXIS1+i,&Enc_axis[i]);//Enc_axis
				//commandhandler("GT_GetEncPos",sRtn);
				//printf("encoder,%f,%f,%f,%f,%f\n",Enc_axis[0],Enc_axis[1],Enc_axis[2],Enc_axis[3],Enc_axis[4]);
			}

			ang_jt_S = Enc_axis[0]* PI/(180* 25600);//弧度值  inverse->ang_j_S
			ang_jt_L = Enc_axis[1]* PI/(180* 22026);//弧度值
			ang_jt_U = Enc_axis[2]* PI/(180* -22026);//弧度值
			ang_jt_R_read = Enc_axis[3]* PI/(180*14545);//encoder value
			ang_jt_T_read = Enc_axis[4]* PI/(-180*13220); 

			//上步角度值
			if(ref_flag)
			{
				pre_ang_S = Inverse->ang_j_S;
				pre_ang_L = Inverse->ang_j_L;
				pre_ang_U = Inverse->ang_j_U;
				pre_ang_R = ang_jt_R;
				pre_ang_T = ang_jt_T;
				ref_flag = false;
			}

			double delt_S;
			double delt_L;
			double delt_U;
			if(start)
			{
				for (int i=0;i<5;i++)
				{
					// 将AXIS1轴设为PT模式
					sRtn = GT_PrfPt(AXIS1 + i, PT_MODE_DYNAMIC);

					// 清空AXIS1轴的FIFO
					sRtn = GT_PtClear(AXIS1+i);
					sRtn = GT_SetPtMemory(AXIS1 + i,0);
				}
				delt_S = Inverse->ang_j_S-pre_ang_S;
				delt_L = Inverse->ang_j_L-pre_ang_L;
				delt_U = Inverse->ang_j_U-pre_ang_U;
				//printf("axcis1=%f,axis2=%f,axis3=%f\r",delt_S,delt_L,delt_U);
				time=100*step;
				if(time<=100){
					time=400;//最小不要超过400
				}
				sRtn = GT_PtData(AXIS1 + 0, delt_S*25600, time,PT_SEGMENT_EVEN);
				sRtn = GT_PtData(AXIS1 + 1, delt_L*(22026), time,PT_SEGMENT_EVEN);
				sRtn = GT_PtData(AXIS1 + 2, delt_U*(-22026), time,PT_SEGMENT_EVEN);
				sRtn = GT_PtData(AXIS1 + 3, (ang_jt_R-pre_ang_R)*14545, time,PT_SEGMENT_EVEN);
				sRtn = GT_PtData(AXIS1 + 4, (ang_jt_T-pre_ang_T) *(-13220), time,PT_SEGMENT_EVEN);

				sRtn = GT_PtStart(1 << (AXIS1 + 0 - 1));			
				sRtn = GT_PtStart(1 << (AXIS1 + 1 - 1));
				sRtn = GT_PtStart(1 << (AXIS1 + 2 - 1));
				sRtn = GT_PtStart(1 << (AXIS1 + 3 - 1));
				sRtn = GT_PtStart(1 << (AXIS1 + 4 - 1));

				pre_ang_S = Inverse->ang_j_S;
				pre_ang_L = Inverse->ang_j_L;
				pre_ang_U = Inverse->ang_j_U;
				pre_ang_R = ang_jt_R;
				pre_ang_T = ang_jt_T;
				start = 0;
			}

			if(kbhit())//保存当前值至数组
			{	
				char c=getch();
				if(c=='s'){
					if(num_samp==50){
						printf("Out of maximum sampling number!Please press q to finish the job.\n");
					}
					else{
						for (int i=0;i<5;i++)
						{
							//读取编码器数值
							sRtn = GT_GetEncPos(AXIS1+i,&Enc_axis[i]);//Enc_axis
						}
						Samp_point[num_samp][1] = Enc_axis[0];
						Samp_point[num_samp][2] = Enc_axis[1];
						Samp_point[num_samp][3] = Enc_axis[2];
						Samp_point[num_samp][4] = Enc_axis[3];
						Samp_point[num_samp][5] = Enc_axis[4];
						printf("s-save %d point q-sampling finish \n",num_samp);
						printf("%f,%f,%f,%f,%f\n",Samp_point[num_samp][1],Samp_point[num_samp][2],Samp_point[num_samp][3],Samp_point[num_samp][4],Samp_point[num_samp][5]);
						num_samp++;
					}
				}
				else if(c=='q'){
					if(num_samp<=2){
						printf("Not enough points!At least 2 points needed.\n");
					}else{
						//save to txt files
						Samp_point[0][1]=num_samp-1;//record how many points does it need
						Samp_point[0][2]=num_samp-1;
						Samp_point[0][3]=num_samp-1;
						Samp_point[0][4]=num_samp-1;
						Samp_point[0][5]=num_samp-1;
						fp[0]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\第1关节.txt","w");
						fp[1]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\第2关节.txt","w");
						fp[2]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\第3关节.txt","w");
						fp[3]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\第4关节.txt","w");
						fp[4]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\第5关节.txt","w");

						for(int i=0;i<5;i++)
						{
							for(int s_num = 0; s_num <num_samp; s_num++)
							{
								if (s_num==0){
									fprintf(fp[i],"%d ",(int)Samp_point[s_num][1+i]);
								}else{
									fprintf(fp[i],"%f ",Samp_point[s_num][1+i]);
								}
							}
						}
						for(int j=0;j<5;j++)
						{
							fclose(fp[j]);
						}
						fp[0]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\axis1.csv","w");
						fp[1]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\axis2.csv","w");
						fp[2]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\axis3.csv","w");
						fp[3]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\axis4.csv","w");
						fp[4]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\axis5.csv","w");

						for(int i=0;i<5;i++)
						{
							for(int s_num = 0; s_num <num_samp; s_num++)
							{
								if (s_num==0){
									fprintf(fp[i],"%d,",(int)Samp_point[s_num][1+i]);
								}else{
									switch(i){
									case 0:
										fprintf(fp[i],"%f,",Samp_point[s_num][1+i]* PI/(180* 25600));
										break;
									case 1:
										fprintf(fp[i],"%f,",Samp_point[s_num][1+i]* PI/(180* 22026));
										break;
									case 2:
										fprintf(fp[i],"%f,",Samp_point[s_num][1+i]* PI/(180* -22026));
										break;
									case 3:
										fprintf(fp[i],"%f,",Samp_point[s_num][1+i]* PI/(180*14545));
										break;
									case 4:
										fprintf(fp[i],"%f,",Samp_point[s_num][1+i]* PI/(-180*13220));
										break;
									default:
										break;
									}
								}
							}
						}
						for(int j=0;j<5;j++)
						{
							fclose(fp[j]);
						}
						printf("exit sampling procedure, press anykey to continue!\n");
						getch();
						break;
					}
				}			
			}
		}
	}
	return num_samp;
}