#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SDX_USE

#ifdef SDX_USE
#include "sds_lib.h"
#endif

#define MAX_LINE 10538		//文件中一共16826行数据
#define LINE_DATA 10		//一行缓冲字符数目
#define MATRIX_MAP_Y 500		//原始地图矩阵
#define MATRIX_MAP_X 605		//原始地图矩阵
#define Origin_lidar_number  541		//原始激光雷达角度数据
#define lidar_number  360		//原始激光雷达角度数据
#define particle_angle_num  311		//粒子角度信息随机个数
#define P_NUMBER  30000//粒子数
#define MATCH_THR 0.7			//匹配度阈值，360*MATCH_THR
#define LIDAR_FILE_READ_LINE  10
#define map_resol 0.02
#define max_score_loop_thr  5 //最高分循环限制，如果连续出现5次，则重新撒粒子，以免陷入死循环
#define angle_resol   0.02		 //角度搜索分辨率0.02为1.5°
#define split_thr     0.1        //如果某粒子分裂数小于0.1*split_num_max, 则认为此粒子是无效粒子
#define p_move_range  50		//粒子移动范围， + -50，这涉及到之后的权值大的粒子分裂的问题
#define RANDOM_MAX 0x7FFFFFFF		//随机数最大值

//my_time用来重置随机种子，此函数不加速
void my_time(unsigned long *rand_seed)
{
	*rand_seed = (unsigned)(time(NULL));
}

////产生随机数的主要函数
//static long my_do_rand(unsigned long *value)
//{
//	/*这个算法保证所产生的值不会超过(2^31 - 1)这里(2^31 - 1)就是 0x7FFFFFFF。而 0x7FFFFFFF
//	等于127773 * (7^5) + 2836,7^5 = 16807。整个算法是通过：t = (7^5 * t) mod (2^31 - 1)这个公式来计算随机值，并且把这次得到的值，作为下次计算的随机种子值。
//	*/
//	long quotient, remainder, t;
//
//	quotient = *value / 127773L;
//	remainder = *value % 127773L;
//	t = 16807L * remainder - 2836L * quotient;
//
//	if (t <= 0)
//		t += 0x7FFFFFFFL;
//	return ((*value = t) % ((unsigned long)RANDOM_MAX + 1));
//}


//
//static unsigned long next = 1;

//返回随机数
//int my_rand(void)
//{
//	return my_do_rand(&next);
//}

//产生随机数的主要函数
void my_rand(unsigned long *value)
{
	/*这个算法保证所产生的值不会超过(2^31 - 1)这里(2^31 - 1)就是 0x7FFFFFFF。而 0x7FFFFFFF
	等于127773 * (7^5) + 2836,7^5 = 16807。整个算法是通过：t = (7^5 * t) mod (2^31 - 1)这个公式来计算随机值，并且把这次得到的值，作为下次计算的随机种子值。
	*/
	long quotient, remainder, t;

	quotient = *value / 127773L;
	remainder = *value % 127773L;
	t = 16807L * remainder - 2836L * quotient;

	if (t <= 0)
		t += 0x7FFFFFFFL;
	*value = (t % ((unsigned long)RANDOM_MAX + 1));
}

//重置时间种子
//void my_srand(void)
//{
//	next = rand_seed;
//}


//读局部激光雷达数据，返回ranges和angles
//struct range_theta get_range_angle(void);
//void get_range_angle(float *MCL_range, float *MCL_angle);
//蒙特卡罗算法，返回计算得到的位置和角度
//void my_MCL_particleLocalization(float *lidar_ranges, float *lidar_angles, int World_map[MATRIX_MAP_Y][MATRIX_MAP_X], int *x_pose, int *y_pose, float *angle_pose, float *match_rate);

//数组大小都不超过1000，使用连续内存来分配，并且使用AXIDMA_SIMPLE来传递数据；大的数组使用AXIDMA_SG
//#pragma SDS data zero_copy(rand_angles_gather[0:particle_angle_num], rand_x_gather[0:MATRIX_MAP_X], rand_y_gather[0:MATRIX_MAP_Y], rand_x[0:P_NUMBER], rand_y[0:P_NUMBER], rand_angle[0:P_NUMBER])
//#pragma SDS data mem_attribute(rand_angles_gather:PHYSICAL_CONTIGUOUS, rand_x_gather:PHYSICAL_CONTIGUOUS, rand_y_gather:PHYSICAL_CONTIGUOUS)
//#pragma SDS data data_mover(rand_angles_gather:AXIDMA_SIMPLE:1, rand_x_gather:AXIDMA_SIMPLE:2, rand_y_gather:AXIDMA_SIMPLE:3, rand_x:AXIDMA_SG:1, rand_y:AXIDMA_SG:2, rand_angle:AXIDMA_SG:3)
void MCL_prepare(float rand_angles_gather[particle_angle_num], short rand_x_gather[MATRIX_MAP_X], short rand_y_gather[MATRIX_MAP_Y], short rand_x[P_NUMBER], short rand_y[P_NUMBER], float rand_angle[P_NUMBER], unsigned long *rand_seed);


//输入：lidar_ranges，lidar_angles，World_map，rand_x，rand_y,rand_angle,score
//输出：max_score_loop，split_number，max_score_pos，max_S
#pragma SDS data zero_copy(World_map[0:MATRIX_MAP_Y*MATRIX_MAP_X], rand_x[0:P_NUMBER], rand_y[0:P_NUMBER], rand_angle[0:P_NUMBER], score[0:P_NUMBER], split_number[0:P_NUMBER])
#pragma SDS data mem_attribute(World_map:PHYSICAL_CONTIGUOUS, rand_x:PHYSICAL_CONTIGUOUS, rand_y:PHYSICAL_CONTIGUOUS, rand_angle:PHYSICAL_CONTIGUOUS, score:PHYSICAL_CONTIGUOUS, split_number:PHYSICAL_CONTIGUOUS)
#pragma SDS data data_mover(World_map:AXIDMA_SG, rand_x:AXIDMA_SIMPLE, rand_y:AXIDMA_SIMPLE, rand_angle:AXIDMA_SIMPLE, score:AXIDMA_SIMPLE, split_number:AXIDMA_SG)
void MCL_process(float lidar_ranges[lidar_number], float lidar_angles[lidar_number], char World_map[MATRIX_MAP_Y][MATRIX_MAP_X], short rand_x[P_NUMBER], short rand_y[P_NUMBER], float rand_angle[P_NUMBER], short score[P_NUMBER], char *max_score_loop, short split_number[P_NUMBER], int *max_score_pos, int *max_S, int *Est_x_pose, int *Est_y_pose, float *Est_angle_pose, float *Est_match_rate, int test_array[10], unsigned long *rand_seed);


//传入参数：max_score_loop，split_number，max_score_pos，split_thr*max_S, rand_angles_gather, rand_x_gather, rand_y_gather, rand_x,rand_y, rand_angle
//输出：rand_x，rand_y，rand_angle
#pragma SDS data zero_copy(split_number[0:P_NUMBER], rand_x[0:P_NUMBER], rand_y[0:P_NUMBER], rand_angle[0:P_NUMBER])
#pragma SDS data mem_attribute(split_number:PHYSICAL_CONTIGUOUS, rand_x:PHYSICAL_CONTIGUOUS, rand_y:PHYSICAL_CONTIGUOUS, rand_angle:PHYSICAL_CONTIGUOUS)
#pragma SDS data data_mover(split_number:AXIDMA_SIMPLE, rand_x:AXIDMA_SG, rand_y:AXIDMA_SG, rand_angle:AXIDMA_SG)
//#pragma SDS data data_mover(split_number:AXIDMA_SIMPLE, rand_x:AXIDMA_SIMPLE, rand_y:AXIDMA_SIMPLE, rand_angle:AXIDMA_SIMPLE)
void MCL_important_sample(char *max_score_loop, short split_number[P_NUMBER], int *max_score_pos, int *max_S, float rand_angles_gather[particle_angle_num], short rand_x_gather[MATRIX_MAP_X], short rand_y_gather[MATRIX_MAP_Y], short rand_x[P_NUMBER], short rand_y[P_NUMBER], float rand_angle[P_NUMBER], unsigned long *rand_seed);


//处理随机数的函数，防止随机数越界
void process_rand_num(short *rand_num, short rand_num_min, short rand_num_max);

//读map文件
void read_map_file(float world_data_x[MAX_LINE], float world_data_y[MAX_LINE]);

//处理全局地图文件
//#pragma SDS data zero_copy(world_data_x[0:MAX_LINE], world_data_y[0:MAX_LINE], World_map[0:MATRIX_MAP_Y*MATRIX_MAP_X])
void process_worldmap(float world_data_x[MAX_LINE], float world_data_y[MAX_LINE], char World_map[MATRIX_MAP_Y][MATRIX_MAP_X]);

//读lidar data文件
void read_lidar_file(float lidar_r[LIDAR_FILE_READ_LINE][Origin_lidar_number]);

//处理lidar data，得到需要的range和angle
//#pragma SDS data zero_copy(lidar_r[0:LIDAR_FILE_READ_LINE*Origin_lidar_number],MCL_range[0:lidar_number], MCL_angle[0:lidar_number])
void process_lidar_file(float lidar_r[LIDAR_FILE_READ_LINE][Origin_lidar_number], float MCL_range[lidar_number], float MCL_angle[lidar_number]);


//全局变量
int i, j, k;			//循环变量
int loop_time = 1;         //记录循环次数
short last_max_score = 0;	//上一次的最高分
int test_number_1;
short test_number_2, test_number_3, test_number_4;

//%----------------------------------------------------------------------------------------------
//%代码功能：读取文件中的x, y数据，构建全局地图。注意：由于我的地图中大于3的数据时不重要的散点，我做了删除此散点
//%工作以缩小栅格地图大小。接下来读取x, y的极值，除以地图分辨率得到地图大小，在对数据逐一判断，既可以得到
//%栅格地图了。
//%%两个重要函数：
//% 1：MCL_range_angle = get_range_angle()：
//%读取局部地图文件，输入无，输出局部地图的ranges, angles, 对应激光雷达扫描到的每个点的极径和角度
//% 2：function myPose = my_MCL_particleLocalization(ranges, angles, map, param)
//% 输入局部地图的ranges, angles, 全局栅格地图，param结构体是地图中原点在栅格地图中的位置等信息
//%蒙特卡罗粒子定位函数，输出得到的位置，具体参见该函数
//%----------------------------------------------------------------------------------------------
//%读全局地图数据，并存入x, y数组中
int main()
{

	float world_data_x[MAX_LINE];								//全局地图中所有的点的x坐标，此处可申请缓存，用完就扔掉
	float world_data_y[MAX_LINE];

	//雷达的数据，由函数得到
	float lidar_angles[lidar_number];
	float lidar_ranges[lidar_number];
	//	float *lidar_angles = (float *)sds_alloc(lidar_number*sizeof(float));
	//	float *lidar_ranges = (float *)sds_alloc(lidar_number*sizeof(float));

	//最终的位置初始化
	int Est_x_pose = 0;
	int Est_y_pose = 0;
	float Est_angle_pose = 0;
	float Est_match_rate = 0;

	float lidar_r[LIDAR_FILE_READ_LINE][Origin_lidar_number] = { 0 };		//存储lidar文件中100行range数据

																			//mcl_prepare arguments
	float rand_angles_gather[particle_angle_num] = { 0 };		//随机角度集合
	short rand_x_gather[MATRIX_MAP_X] = { 0 };				//随机X坐标集合
	short rand_y_gather[MATRIX_MAP_Y] = { 0 };				//随机Y坐标集合

#ifdef SDX_USE															//#ifdef SDX_USE
	short *rand_x = (short *)sds_alloc(P_NUMBER * sizeof(short));
	short *rand_y = (short *)sds_alloc(P_NUMBER * sizeof(short));
	float *rand_angle = (float *)sds_alloc(P_NUMBER * sizeof(float));
	short *score = (short *)sds_alloc(P_NUMBER * sizeof(short));
	short *split_number = (short *)sds_alloc(P_NUMBER * sizeof(short));
	char *World_map = (char *)sds_alloc(MATRIX_MAP_Y*MATRIX_MAP_X * sizeof(char));
#else
	short rand_x[P_NUMBER];							//处理随机数后得到的粒子X坐标
	short rand_y[P_NUMBER];
	float rand_angle[P_NUMBER];
	short score[P_NUMBER] = { 0 };				//每个粒子的得分，即匹配度
	short split_number[P_NUMBER] = { 0 };		//权值对应分裂的个数
	char World_map[MATRIX_MAP_Y][MATRIX_MAP_X] = { 0 };			//全局地图矩阵
#endif

	char max_score_loop = 1;    //同一个最大值循环次数，超过阈值则重新撒粒子
	int max_score_pos = 0;	//最高得分在粒子数组中的位置
	int max_S;				//由权值来计算分裂数，这是最大分裂数

							//
	char run_MCL_fun = 1;		//循环驱动

	int test_array[10] = { 0 };		//调试用数组

	unsigned long my_rand_number_seed = 1;

	read_map_file(world_data_x, world_data_y);
	process_worldmap(world_data_x, world_data_y, World_map);


	/*-----------------------------------------------------*/
	//调用函数读取激光雷达的数据并读入lidar_angles，lidar_ranges
	//get_range_angle(lidar_ranges, lidar_angles);

	read_lidar_file(lidar_r);
	process_lidar_file(lidar_r, lidar_ranges, lidar_angles);


	my_time(&my_rand_number_seed);		//将时间种子赋给my_rand_number_seed

	MCL_prepare(rand_angles_gather, rand_x_gather, rand_y_gather, rand_x, rand_y, rand_angle, &my_rand_number_seed);

	while (run_MCL_fun)
	{
		my_time(&my_rand_number_seed);		//将时间种子赋给my_rand_number_seed
		MCL_process(lidar_ranges, lidar_angles, World_map, rand_x, rand_y, rand_angle, score, &max_score_loop, split_number, &max_score_pos, &max_S, &Est_x_pose, &Est_y_pose, &Est_angle_pose, &Est_match_rate, test_array, &my_rand_number_seed);
		printf("Estimate %d times pose are:  \n x = %d \n y = %d \n angle = %f\n match_rate = %f \n", loop_time, Est_x_pose, Est_y_pose, Est_angle_pose, Est_match_rate);
		loop_time++;
		printf("rand test_number is %d  %d  %d  %d\n", test_array[0], test_array[1], test_array[2], test_array[3]);
		if (Est_match_rate > MATCH_THR)
		{
			run_MCL_fun = 0;
		}
		//如果匹配点最多没有到达阈值，则重采样
		else
		{
			my_time(&my_rand_number_seed);		//将时间种子赋给my_rand_number_seed
			printf("rand time seed is %d\n", my_rand_number_seed);
			MCL_important_sample(&max_score_loop, split_number, &max_score_pos, &max_S, rand_angles_gather, rand_x_gather, rand_y_gather, rand_x, rand_y, rand_angle, &my_rand_number_seed);
			printf("finish important sample\n");
		}
	}

	//	char find_times = 0;
	//	for (find_times = 0; find_times < 10; find_times ++)
	//	{
	//		#pragma HLS loop_tripcount min=1 max=10		//指明循坏次数的最大值和最小值
	//		MCL_process(lidar_ranges, lidar_angles, World_map, rand_x, rand_y, rand_angle, score, &max_score_loop, split_number, &max_score_pos, &max_S, &Est_x_pose, &Est_y_pose, &Est_angle_pose, &Est_match_rate);
	//		if (Est_match_rate > MATCH_THR)
	//		{
	//			break;
	//		}
	//		//如果匹配点最多没有到达阈值，则重采样
	//		else
	//		{
	//			my_time();
	//			MCL_important_sample(&max_score_loop, split_number, &max_score_pos, &max_S, rand_angles_gather, rand_x_gather, rand_y_gather, rand_x, rand_y, rand_angle);
	//		}
	//	}




	printf("\n\n--------------Successful!!!--------------\n\n\n");
	printf("Final Estimate Pose are:  \n x = %d \n y = %d \n angle = %f\n match_rate = %f", Est_x_pose, Est_y_pose, Est_angle_pose, Est_match_rate);

	//从键盘得到输入，用于保留终端输出框
	getchar();
#pragma  message("final")
	return 0;
}

void process_rand_num(short *rand_num, short rand_num_min, short rand_num_max)
{
	if (*rand_num > rand_num_max)
	{
		*rand_num = rand_num_max;
	}
	else if (*rand_num < rand_num_min)
	{
		*rand_num = rand_num_min;
	}
}

void process_worldmap(float world_data_x[MAX_LINE], float world_data_y[MAX_LINE], char World_map[MATRIX_MAP_Y][MATRIX_MAP_X])
{
	float world_x_min = 0;		//全局地图中的x,y最大最小值，用于计算地图矩阵大小
	float world_x_max = 0;
	float world_y_min = 0;
	float world_y_max = 0;

	int map_y;				//计算地图y,x的大小
	int map_x;
	int x_data, y_data;		//计算地图绝对坐标转换到地图矩阵之中的数据

							//去除地图中大于3的散点
	for (i = 0; i < MAX_LINE - 1; i++)
	{
		if (world_data_y[i] > 3)
		{
			world_data_x[i] = world_data_x[1];
			world_data_y[i] = world_data_y[1];
		}
	}


	//将坐标转化到矩阵中去
	for (i = 0; i < MAX_LINE - 1; i++)
	{
		if (world_data_y[i] > world_y_max)
		{
			world_y_max = world_data_y[i];
		}
		if (world_data_y[i] < world_y_min)
		{
			world_y_min = world_data_y[i];
		}
		if (world_data_x[i] > world_x_max)
		{
			world_x_max = world_data_x[i];
		}
		if (world_data_x[i] < world_x_min)
		{
			world_x_min = world_data_x[i];
		}
	}

	//计算地图矩阵的x,y，	x,y定义同图片矩阵
	map_y = ceil((world_y_max - world_y_min) / map_resol) + 1;
	map_x = ceil((world_x_max - world_x_min) / map_resol) + 1;

	for (i = 0; i < MAX_LINE - 1; i++)
	{
		//+1防止出现数组下标为0的情况
		x_data = round((world_data_x[i] - world_x_min) / map_resol + 1);
		y_data = round((world_y_max - world_data_y[i]) / map_resol + 1);
		World_map[y_data][x_data] = 1;
	}

	////地图中起始位置0, 0在矩阵中的坐标
	//int zero_x_data, zero_y_data;
	//zero_x_data = round((0 - world_x_min) / map_resol + 1);
	//zero_y_data = round((world_y_max - 0) / map_resol + 1);
}


void read_lidar_file(float lidar_r[LIDAR_FILE_READ_LINE][Origin_lidar_number])
{
	FILE *fp;            /*文件指针*/
	if ((fp = fopen("Range-data-2018-04-16-20-19.txt", "r")) == NULL)
	{
		perror("fail to read");
		exit(1);
	}

	for (j = 0; j < LIDAR_FILE_READ_LINE; j++)
	{
		for (i = 0; i < Origin_lidar_number; i++)
		{
			fscanf(fp, "%f", &lidar_r[k][i]);
			//printf("range = %.3f\n", r[k][i]);
		}
		k = k + 1;
		//printf("x = %sy = %s", buf_x , buf_y);
	}
	fclose(fp);
}

//处理lidar data，得到需要的range和angle
void process_lidar_file(float lidar_r[LIDAR_FILE_READ_LINE][Origin_lidar_number], float MCL_range[lidar_number], float MCL_angle[lidar_number])
{
	float r_aver[lidar_number] = { 0 };			//几帧激光雷达数据取平均得到的数据储存于此
	float theta[lidar_number] = { 0 };			//0-180°范围内的角度信息
	float r_aver_old[Origin_lidar_number] = { 0 };	//激光雷达存储中间数组
	double theta_old[Origin_lidar_number] = { 0 };	//角度存储中间数组
	double theta_old_resol = 0.00872664619237;		//角度的分辨率
	theta_old[0] = -2.35619449615;					//角度的起始值
	short first_line = 5;							//取first_line-last_line数据平均值
	short last_line = 7;
	k = 0;			//循环变量

	for (i = 0; i < Origin_lidar_number; i++)
	{
		for (j = first_line; j < last_line + 1; j++)
		{
			r_aver_old[i] = lidar_r[j][i] + r_aver_old[i];
		}
		r_aver_old[i] = r_aver_old[i] / (last_line - first_line + 1);
		theta_old[i] = -2.35619449615 + i * theta_old_resol;
	}

	//剔除多余角度数据，只取0-180度数据
	j = 0;
	for (i = 0; i < Origin_lidar_number; i++)
	{
		//pi/2 = 1.570796326794897;;;;fabs函数的参数与返回值均为实值
		if (fabs(theta_old[i]) < 1.570796326794897)
		{
			MCL_range[j] = r_aver_old[i];
			MCL_angle[j] = theta_old[i];
			j = j + 1;
		}
	}
}


void MCL_prepare(float rand_angles_gather[particle_angle_num], short rand_x_gather[MATRIX_MAP_X], short rand_y_gather[MATRIX_MAP_Y], short rand_x[P_NUMBER], short rand_y[P_NUMBER], float rand_angle[P_NUMBER], unsigned long *rand_seed)
{
	unsigned long function_rand_number = *rand_seed;
	for (i = 0; i < particle_angle_num; i++)
	{
		rand_angles_gather[i] = i * angle_resol;
	}
	//创建x坐标矩阵，用于随机数挑选
	for (i = 0; i < MATRIX_MAP_X; i++)
	{
		rand_x_gather[i] = i + 1;
	}
	//创建y坐标矩阵，用于随机数挑选
	for (i = 0; i < MATRIX_MAP_Y; i++)
	{
		rand_y_gather[i] = i + 1;
	}

	//随机数给rand_x,y,angle赋值
	//-------------------数组占用内存太大，后续需处理------------------------
	/*srand((unsigned)time(NULL));*/
	for (i = 0; i < P_NUMBER; i++)
	{
		my_rand(&function_rand_number);
		rand_x[i] = rand_x_gather[function_rand_number % MATRIX_MAP_X];
		my_rand(&function_rand_number);
		rand_y[i] = rand_y_gather[function_rand_number % MATRIX_MAP_Y];
		my_rand(&function_rand_number);
		rand_angle[i] = rand_angles_gather[function_rand_number % particle_angle_num];
		//printf("Random numbers are: %d	%d	%d\n", rand_x[i], rand_y[i] , rand_angle[i]);
	}

}


//输入：lidar_ranges，lidar_angles，World_map，rand_x，rand_y,rand_angle,score
//输出：max_score_loop，split_number，max_score_pos，max_S
void read_map_file(float world_data_x[MAX_LINE], float world_data_y[MAX_LINE])
{
	char buf_x[LINE_DATA];  /*缓冲区*/
	char buf_y[LINE_DATA];  /*缓冲区*/
	FILE *fp_x, *fp_y;            /*文件指针*/
	int len;             /*行字符个数*/
						 //打开文件
	if ((fp_x = fopen("x_0416.txt", "r")) == NULL || (fp_y = fopen("y_0416.txt", "r")) == NULL)
	{
		perror("fail to read");
		exit(1);
	}

	//读取全局地图x,y坐标
	while ((fgets(buf_x, LINE_DATA, fp_x) != NULL) && (fgets(buf_y, LINE_DATA, fp_y) != NULL))
	{
		world_data_x[i] = atof(buf_x);
		world_data_y[i] = atof(buf_y);
		i = i + 1;
		//printf("x = %sy = %s", buf_x , buf_y);
	}
	fclose(fp_x);
	fclose(fp_y);
}

void MCL_process(float lidar_ranges[lidar_number], float lidar_angles[lidar_number], char World_map[MATRIX_MAP_Y][MATRIX_MAP_X], short rand_x[P_NUMBER], short rand_y[P_NUMBER], float rand_angle[P_NUMBER], short score[P_NUMBER], char *max_score_loop, short split_number[P_NUMBER], int *max_score_pos, int *max_S, int *Est_x_pose, int *Est_y_pose, float *Est_angle_pose, float *Est_match_rate, int test_array[10], unsigned long *rand_seed)
{

	//	#pragma HLS allocation instances=select limit=512 operation		//限制select表达式最多512个
	//	#pragma HLS allocation instances=add limit=256 operation		//
	//	#pragma HLS allocation instances=icmp limit=256 operation		//
	//	#pragma HLS allocation instances=or limit=128 operation		//
	//	#pragma HLS ALLOCATION instances=mul limit=200 operation
	float angle, range;			//临时变量，用于计算occ_x,occ_y
	int occ_x, occ_y;			//局部地图以粒子位置为机器人位置，得到的x,y值，后续将判断地图上此点是否存在，存在则加1分
	int rand_x_max;				////储存最大得分的位置信息
	int rand_y_max;
	float rand_angle_max;

	int sum_score = 0;		//计算得分总和
	int max_score = 0;		//计算最高得分

	float rand_angle_store;
	short rand_x_store;
	short rand_y_store;

	unsigned long function_rand_number = *rand_seed;		//得到时间种子

															//将打分系统清零
															//*********LOOP1
	for (i = 0; i < P_NUMBER; i++)
	{
#pragma HLS loop_tripcount min=100000 max=200000		//指明循坏次数的最大值和最小值
		//#pragma HLS unroll factor=50
#pragma HLS pipeline II=1
		score[i] = 0;
	}

	//对于每个粒子
	//*********LOOP2
	for (i = 0; i < P_NUMBER; i++)
	{
#pragma HLS loop_tripcount min=100000 max=1000000		//指明循坏次数的最大值和最小值
		//对每个粒子进行打分，如果局部地图的点转换后在全局地图中也有，则加一分
		////*******************这个地方可以加pipeline吗？
		rand_angle_store = rand_angle[i];		//储存循环中需要的值避免重复读取数据
		rand_x_store = rand_x[i];
		rand_y_store = rand_y[i];

		//*********LOOP2.1
		for (j = 0; j < lidar_number; j++)
		{
#pragma HLS loop_tripcount min=360 max=2000		//指明循坏次数的最大值和最小值
#pragma HLS pipeline II=1	 					//此循环内执行流水线操作
			//			#pragma HLS unroll factor=5			//在这里小小的用一下unroll,以满足时序要求
			//			#pragma HLS dependence variable=score intra false		//由于score的存在，需要给个提示

			////对于每个粒子，将局 部扫描地图带入，得到该局部地图在全局地图中对应的坐标。这里其实是一个坐标转换工作
			//			#pragma HLS RESOURCE variable=angle core=FAddSub

			angle = lidar_angles[j] + rand_angle_store;
			range = lidar_ranges[j];
			occ_x = ceil(range*cos(angle) / map_resol + rand_x_store);
			occ_y = ceil(range*sin(angle) / map_resol + rand_y_store);

			//			#pragma HLS RESOURCE variable=occ_y core=Cmp
			//			#pragma HLS RESOURCE variable=occ_x core=Cmp
			if (occ_y < MATRIX_MAP_Y && occ_x < MATRIX_MAP_X && occ_x >= 0 && occ_y >= 0)
			{
				if (World_map[occ_y][occ_x] == 1)
				{
					score[i] = score[i] + 1;
				}
			}

		}

	}

	/////////////////////////////////////////////////////////////////////
	sum_score = 0;		//计算得分总和
	max_score = 0;		//计算最高得分
	*max_score_pos = 0;	//最高得分在粒子数组中的位置
						//计算总得分，及最大分数，最大分数的位置
						//*********LOOP3
	for (i = 0; i < P_NUMBER; i++)
	{
#pragma HLS loop_tripcount min=100000 max=1000000		//指明循坏次数的最大值和最小值,没啥用
		//		#pragma HLS unroll factor=50
#pragma HLS pipeline II = 1					//此循环内执行流水线操作
		//		#pragma HLS dependence variable=sum_score intra false		//由于sum_score的存在，需要给个提示

		sum_score = sum_score + score[i];
		if (score[i] > max_score)
		{
			max_score = score[i];
			*max_score_pos = i;
		}
	}

	//储存最大得分的位置信息
	rand_x_max = rand_x[*max_score_pos];
	rand_y_max = rand_y[*max_score_pos];
	rand_angle_max = rand_angle[*max_score_pos];


	//判断最大分数出现的次数
	if (last_max_score == max_score)
	{
		*max_score_loop = *max_score_loop + 1;
	}
	else
	{
		*max_score_loop = 1;
		last_max_score = max_score;
	}

	//根据得分和总分算出权值对应分裂的个数
	//*********LOOP4
	for (i = 0; i < P_NUMBER; i++)
	{
#pragma HLS loop_tripcount min=100000 max=1000000		//指明循坏次数的最大值和最小值
#pragma HLS pipeline II=1						//此循环内执行流水线操作
		split_number[i] = round((score[i] * P_NUMBER) / (sum_score*1.0));
	}

	*max_S = round(max_score * P_NUMBER / sum_score);

	my_rand(&function_rand_number);
	test_array[0] = function_rand_number;
	test_array[1] = function_rand_number % (2 * p_move_range);
	test_array[2] = ceil(-p_move_range + test_array[1]);
	test_array[3] = ceil(-p_move_range + test_array[1]);
	process_rand_num(&test_array[3], -p_move_range, p_move_range);

	//计算匹配度，并与阈值比较
	*Est_angle_pose = rand_angle_max;
	*Est_x_pose = rand_x_max;
	*Est_y_pose = rand_y_max;
	*Est_match_rate = max_score / 360.0;
}


//重采样函数
//传入参数：max_score_loop，split_number，max_score_pos，split_thr*max_S, rand_angles_gather, rand_x_gather, rand_y_gather, rand_x,rand_y, rand_angle
//输出：rand_x，rand_y，rand_angle
void MCL_important_sample(char *max_score_loop, short split_number[P_NUMBER], int *max_score_pos, int *max_S, float rand_angles_gather[particle_angle_num], short rand_x_gather[MATRIX_MAP_X], short rand_y_gather[MATRIX_MAP_Y], short rand_x[P_NUMBER], short rand_y[P_NUMBER], float rand_angle[P_NUMBER], unsigned long *rand_seed)
{
	//	#pragma HLS allocation instances=select limit=512 operation		//限制select表达式最多512个
	//	#pragma HLS allocation instances=add limit=256 operation		//
	//	#pragma HLS allocation instances=icmp limit=256 operation		//
	//	#pragma HLS allocation instances=or limit=128 operation		//
	//	#pragma HLS ALLOCATION instances=mul limit=200 operation
#pragma HLS allocation instances=process_rand_num limit=10 function
#pragma HLS allocation instances=my_rand limit=10 function

	int P_num;								//P_num用于给所有粒子赋值的驱动变量
	short rand_num_1, rand_num_2, rand_num_3, rand_num_temp;				//随机数，用于在随机数组里取数值
	int k_start;		//最后补全粒子需要用

	float rand_angle_store;
	short rand_x_store;
	short rand_y_store;
	int max_score_pos_store = *max_score_pos;

	short sample_reprepare;	//重采样接口

	short split_num_thr;
	short split_num_i_store;

	unsigned long function_rand_number = *rand_seed;	//得到时间种子

														//对于权重最大的粒子，判断此最高分出现的次数以免陷入误区。保存该粒子信息, 确保每次循环后得分不会降低
	if (*max_score_loop < max_score_loop_thr)
	{
		rand_x[0] = rand_x[max_score_pos_store];
		rand_y[0] = rand_y[max_score_pos_store];
		rand_angle[0] = rand_angle[max_score_pos_store];
		P_num = 1;
		sample_reprepare = 0;
	}
	else
	{
		sample_reprepare = 1;
		P_num = 0;
	}


	//如果连续thr次同样最大值，则重新随机撒粒子
	if (sample_reprepare > 0)
	{
		//随机数给rand_x,y,angle赋值
		for (i = 0; i < P_NUMBER; i++)
		{
			my_rand(&function_rand_number);
			rand_x[i] = rand_x_gather[function_rand_number % MATRIX_MAP_X];
			my_rand(&function_rand_number);
			rand_y[i] = rand_y_gather[function_rand_number % MATRIX_MAP_Y];
			my_rand(&function_rand_number);
			rand_angle[i] = rand_angles_gather[function_rand_number % particle_angle_num];
		}
	}
	else
	{
		// %对每个粒子进行处理，重采样
		//**********LOOP1
		split_num_i_store = split_number[i];
		split_num_thr = ceil(split_thr * (*max_S));

		for (i = 0; i < P_NUMBER; i++)
		{
#pragma HLS loop_tripcount min=100000 max=1000000		//指明循坏次数的最大值和最小值

			if (split_num_i_store > split_num_thr && i != max_score_pos_store)
			{
				rand_angle_store = rand_angle[i];		//储存循环中需要的值避免重复读取数据
				rand_x_store = rand_x[i];
				rand_y_store = rand_y[i];
				k_start = P_num;

				//%分裂随机数计算
				//%分裂后的x, y的随机值在原数据周围 + -p_move_range范围内随机选取
				//**********LOOP1.1
				for (j = 0; j < split_num_i_store; j++)
				{
#pragma HLS loop_tripcount min=10 max=2000		//指明循坏次数的最大值和最小值
#pragma HLS pipeline II=40	 					//此循环内执行流水线操作

					my_rand(&function_rand_number);
					rand_num_temp = (function_rand_number % (2 * p_move_range));
					rand_num_1 = ceil(rand_x_store - p_move_range + rand_num_temp);
					process_rand_num(&rand_num_1, 0, MATRIX_MAP_X - 1);
					rand_x[P_num] = rand_x_gather[rand_num_1];

					P_num = P_num + 1;
				}

				P_num = k_start;
				for (j = 0; j < split_num_i_store; j++)
				{
#pragma HLS loop_tripcount min=10 max=2000		//指明循坏次数的最大值和最小值
#pragma HLS pipeline II=40	 					//此循环内执行流水线操作

					my_rand(&function_rand_number);
					rand_num_temp = (function_rand_number % (2 * p_move_range));
					rand_num_2 = ceil(rand_y_store - p_move_range + rand_num_temp);
					process_rand_num(&rand_num_2, 0, MATRIX_MAP_Y - 1);
					rand_y[P_num] = rand_y_gather[rand_num_2];

					P_num = P_num + 1;
				}

				P_num = k_start;
				for (j = 0; j < split_num_i_store; j++)
				{
#pragma HLS loop_tripcount min=10 max=2000		//指明循坏次数的最大值和最小值
#pragma HLS pipeline II=40	 					//此循环内执行流水线操作

					my_rand(&function_rand_number);
					rand_num_temp = (function_rand_number % (2 * p_move_range));
					rand_num_3 = ceil(rand_angle_store / angle_resol + 1 - p_move_range + rand_num_temp);
					process_rand_num(&rand_num_3, 0, particle_angle_num - 1);
					rand_angle[P_num] = rand_angles_gather[rand_num_3];

					P_num = P_num + 1;
				}

			}
		}

		//%粒子分裂时，特意没有处理分裂数最多的粒子，在此处处理，并将粒子数补满
		k_start = P_num;
		rand_angle_store = rand_angle[max_score_pos_store];		//储存循环中需要的值避免重复读取数据
		rand_x_store = rand_x[max_score_pos_store];
		rand_y_store = rand_y[max_score_pos_store];

		//**********LOOP2
		for (k = k_start; k < P_NUMBER; k++)
		{
#pragma HLS loop_tripcount min=10 max=1000000		//指明循坏次数的最大值和最小值
#pragma HLS pipeline II=40	 					//此循环内执行流水线操作

			my_rand(&function_rand_number);
			rand_num_temp = (function_rand_number % (2 * p_move_range));
			rand_num_1 = ceil(rand_x_store - p_move_range + rand_num_temp);
			process_rand_num(&rand_num_1, 0, MATRIX_MAP_X - 1);
			rand_x[P_num] = rand_x_gather[rand_num_1];

			P_num = P_num + 1;
		}

		P_num = k_start;
		for (k = k_start; k < P_NUMBER; k++)
		{
#pragma HLS loop_tripcount min=10 max=1000000		//指明循坏次数的最大值和最小值
#pragma HLS pipeline II=40	 					//此循环内执行流水线操作

			my_rand(&function_rand_number);
			rand_num_temp = (function_rand_number % (2 * p_move_range));
			rand_num_2 = ceil(rand_y_store - p_move_range + rand_num_temp);
			process_rand_num(&rand_num_2, 0, MATRIX_MAP_Y - 1);
			rand_y[P_num] = rand_y_gather[rand_num_2];

			P_num = P_num + 1;
		}

		P_num = k_start;
		for (k = k_start; k < P_NUMBER; k++)
		{
#pragma HLS loop_tripcount min=10 max=1000000		//指明循坏次数的最大值和最小值
#pragma HLS pipeline II=40	 					//此循环内执行流水线操作

			my_rand(&function_rand_number);
			rand_num_temp = (function_rand_number % (2 * p_move_range));
			rand_num_3 = ceil(rand_angle_store / angle_resol + 1 - p_move_range + rand_num_temp);
			process_rand_num(&rand_num_3, 0, particle_angle_num - 1);
			rand_angle[P_num] = rand_angles_gather[rand_num_3];

			P_num = P_num + 1;
		}
	}
}
