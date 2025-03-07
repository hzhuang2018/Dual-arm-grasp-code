#pragma once
#ifndef FUZZY_PID_H_
#define FUZZY_PID_H_
#include<iostream>
#include<string>
using std::string;
using std::cout;
using std::cin;
using std::endl;
class FuzzyPID
{
public:
	const static int N = 7;
private:
	float target;  //ϵͳ�Ŀ���Ŀ��
	float actual;  //������õ�ʵ��ֵ
	float e;       //���
	float e_pre_1; //��һ�ε����
	float e_pre_2; //���ϴε����
	float de;      //���ı仯��
	float emax;    //��������������
	float demax;   //���绯�ʻ������������
	float delta_Kp_max;   //delta_kp���������
	float delta_Ki_max;   //delta_ki�������
	float delta_Kd_max;   //delta_kd�������
	float Ke;      //Ke=n/emax,��������Ϊ[-3,-2,-1,0,1,2,3]
	float Kde;     //Kde=n/demax,��������Ϊ[-3,-2,-1,0,1,2,3]
	float Ku_p;    //Ku_p=Kpmax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
	float Ku_i;    //Ku_i=Kimax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
	float Ku_d;    //Ku_d=Kdmax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
	int Kp_rule_matrix[N][N];//Kpģ���������
	int Ki_rule_matrix[N][N];//Kiģ���������
	int Kd_rule_matrix[N][N];//Kdģ���������
	string mf_t_e;       //e�������Ⱥ�������
	string mf_t_de;      //de�������Ⱥ�������
	string mf_t_Kp;      //kp�������Ⱥ�������
	string mf_t_Ki;      //ki�������Ⱥ�������
	string mf_t_Kd;      //kd�������Ⱥ�������
	float* e_mf_paras; //���������Ⱥ����Ĳ���
	float* de_mf_paras;//����ƫ�������Ⱥ����Ĳ���
	float* Kp_mf_paras; //kp�������Ⱥ����Ĳ���
	float* Ki_mf_paras; //ki�������Ⱥ����Ĳ���
	float* Kd_mf_paras; //kd�������Ⱥ����Ĳ���
	float Kp;
	float Ki;
	float Kd;
	float A;
	float B;
	float C;
	void showMf(const string& type, float* mf_paras);      //��ʾ�����Ⱥ�������Ϣ
	void setMf_sub(const string& type, float* paras, int n);//����ģ�������Ⱥ������Ӻ���
public:
	FuzzyPID(float e_max, float de_max, float kp_max, float ki_max, float kd_max, float Kp0, float Ki0, float Kd0);
	FuzzyPID(float* fuzzyLimit, float* pidInitVal);
	~FuzzyPID();
	float trimf(float x, float a, float b, float c);          //���������Ⱥ���
	float gaussmf(float x, float ave, float sigma);          //��̬�����Ⱥ���
	float trapmf(float x, float a, float b, float c, float d); //���������Ⱥ���
	void setMf(const string& mf_type_e, float* e_mf,
		const string& mf_type_de, float* de_mf,
		const string& mf_type_Kp, float* Kp_mf,
		const string& mf_type_Ki, float* Ki_mf,
		const string& mf_type_Kd, float* Kd_mf);	//����ģ�������Ⱥ����Ĳ���
	void setRuleMatrix(int kp_m[N][N], int ki_m[N][N], int kd_m[N][N]);  //����ģ������
	float realize(float t, float a);              //ʵ��ģ������
	void showInfo();                                      //��ʾ��ģ������������Ϣ
};

#endif
