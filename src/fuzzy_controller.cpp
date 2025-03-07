#include"fuzzy_controller.h"

FuzzyPID::FuzzyPID(float e_max, float de_max, float kp_max, float ki_max, float kd_max, float Kp0, float Ki0, float Kd0) :
	target(0), actual(0), emax(e_max), demax(de_max), delta_Kp_max(kp_max), delta_Ki_max(ki_max), delta_Kd_max(kd_max), e_mf_paras(NULL), de_mf_paras(NULL),
	Kp_mf_paras(NULL), Ki_mf_paras(NULL), Kd_mf_paras(NULL)
{
	e = target - actual;
	e_pre_1 = 0;
	e_pre_2 = 0;
	de = e - e_pre_1;
	Ke = (N / 2) / emax;
	Kde = (N / 2) / demax;
	Ku_p = delta_Kp_max / (N / 2);
	Ku_i = delta_Ki_max / (N / 2);
	Ku_d = delta_Kd_max / (N / 2);
	mf_t_e = "No type";
	mf_t_de = "No type";
	mf_t_Kp = "No type";
	mf_t_Ki = "No type";
	mf_t_Kd = "No type";
	Kp = Kp0;
	Ki = Ki0;
	Kd = Kd0;
	A = Kp + Ki + Kd;
	B = -2 * Kd - Kp;
	C = Kd;
}

FuzzyPID::FuzzyPID(float* fuzzyLimit, float* pidInitVal)
{
	target = 0;
	actual = 0;
	e = 0;
	e_pre_1 = 0;
	e_pre_2 = 0;
	de = e - e_pre_1;
	emax = fuzzyLimit[0];
	demax = fuzzyLimit[1];
	delta_Kp_max = fuzzyLimit[2];
	delta_Ki_max = fuzzyLimit[3];
	delta_Kd_max = fuzzyLimit[4];
	Ke = (N / 2) / emax;
	Kde = (N / 2) / demax;
	Ku_p = delta_Kp_max / (N / 2);
	Ku_i = delta_Ki_max / (N / 2);
	Ku_d = delta_Kd_max / (N / 2);
	mf_t_e = "No type";
	mf_t_de = "No type";
	mf_t_Kp = "No type";
	mf_t_Ki = "No type";
	mf_t_Kd = "No type";
	e_mf_paras = NULL;
	de_mf_paras = NULL;
	Kp_mf_paras = NULL;
	Ki_mf_paras = NULL;
	Kd_mf_paras = NULL;

	Kp = pidInitVal[0];
	Ki = pidInitVal[1];
	Kd = pidInitVal[2];
	A = Kp + Ki + Kd;
	B = -2 * Kd - Kp;
	C = Kd;
}

FuzzyPID::~FuzzyPID()
{
	delete[] e_mf_paras;
	delete[] de_mf_paras;
	delete[] Kp_mf_paras;
	delete[] Ki_mf_paras;
	delete[] Kd_mf_paras;
}
//���������Ⱥ���
float FuzzyPID::trimf(float x, float a, float b, float c)
{
	float u;
	if (x >= a && x <= b)
		u = (x - a) / (b - a);
	else if (x > b && x <= c)
		u = (c - x) / (c - b);
	else
		u = 0.0;
	return u;

}

//����ģ������Matrix
void FuzzyPID::setRuleMatrix(int kp_m[N][N], int ki_m[N][N], int kd_m[N][N])
{
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
		{
			Kp_rule_matrix[i][j] = kp_m[i][j];
			Ki_rule_matrix[i][j] = ki_m[i][j];
			Kd_rule_matrix[i][j] = kd_m[i][j];
		}
}
//����ģ�������Ⱥ������Ӻ���
void FuzzyPID::setMf_sub(const string& type, float* paras, int n)
{
	int N_mf_e, N_mf_de, N_mf_Kp, N_mf_Ki, N_mf_Kd;
	switch (n)
	{
	case 0:
		if (type == "trimf" || type == "gaussmf" || type == "trapmf")
			mf_t_e = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_t_e == "trimf")
			N_mf_e = 3;

		e_mf_paras = new float[N * N_mf_e];
		for (int i = 0; i < N * N_mf_e; i++)
			e_mf_paras[i] = paras[i];
		break;

	case 1:
		if (type == "trimf" || type == "gaussmf" || type == "trapmf")
			mf_t_de = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_t_de == "trimf")
			N_mf_de = 3;

		de_mf_paras = new float[N * N_mf_de];
		for (int i = 0; i < N * N_mf_de; i++)
			de_mf_paras[i] = paras[i];
		break;

	case 2:
		if (type == "trimf" || type == "gaussmf" || type == "trapmf")
			mf_t_Kp = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_t_Kp == "trimf")
			N_mf_Kp = 3;

		Kp_mf_paras = new float[N * N_mf_Kp];
		for (int i = 0; i < N * N_mf_Kp; i++)
			Kp_mf_paras[i] = paras[i];
		break;

	case 3:
		if (type == "trimf" || type == "gaussmf" || type == "trapmf")
			mf_t_Ki = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_t_Ki == "trimf")
			N_mf_Ki = 3;

		Ki_mf_paras = new float[N * N_mf_Ki];
		for (int i = 0; i < N * N_mf_Ki; i++)
			Ki_mf_paras[i] = paras[i];
		break;

	case 4:
		if (type == "trimf" || type == "gaussmf" || type == "trapmf")
			mf_t_Kd = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_t_Kd == "trimf")
			N_mf_Kd = 3;

		Kd_mf_paras = new float[N * N_mf_Kd];
		for (int i = 0; i < N * N_mf_Kd; i++)
			Kd_mf_paras[i] = paras[i];
		break;

	default: break;
	}
}
//����ģ�������Ⱥ��������ͺͲ���
void FuzzyPID::setMf(const string& mf_type_e, float* e_mf,
	const string& mf_type_de, float* de_mf,
	const string& mf_type_Kp, float* Kp_mf,
	const string& mf_type_Ki, float* Ki_mf,
	const string& mf_type_Kd, float* Kd_mf)
{
	setMf_sub(mf_type_e, e_mf, 0);
	setMf_sub(mf_type_de, de_mf, 1);
	setMf_sub(mf_type_Kp, Kp_mf, 2);
	setMf_sub(mf_type_Ki, Ki_mf, 3);
	setMf_sub(mf_type_Kd, Kd_mf, 4);
}
//ʵ��ģ������
float FuzzyPID::realize(float t, float a)
{
	float u_e[N], u_de[N], u_u[N];
	int u_e_index[3], u_de_index[3];//����һ��������༤��3��ģ���Ӽ�
	float delta_Kp, delta_Ki, delta_Kd;
	float delta_u;
	target = t;
	actual = a;
	e = target - actual;
	de = e - e_pre_1;
	e = Ke * e;
	de = Kde * de;
	/* �����eģ����*/
	int j = 0;
	for (int i = 0; i < N; i++)
	{
		if (mf_t_e == "trimf")
			u_e[i] = trimf(e, e_mf_paras[i * 3], e_mf_paras[i * 3 + 1], e_mf_paras[i * 3 + 2]);//eģ��������������������

		if (u_e[i] != 0)
			u_e_index[j++] = i;                //�洢�������ģ���Ӽ����±꣬���Լ�С������
	}
	for (; j < 3; j++)u_e_index[j] = 0;             //����Ŀռ���0

	/*�����仯��deģ����*/
	j = 0;
	for (int i = 0; i < N; i++)
	{
		if (mf_t_de == "trimf")
			u_de[i] = trimf(de, de_mf_paras[i * 3], de_mf_paras[i * 3 + 1], de_mf_paras[i * 3 + 2]);//deģ��������������������

		if (u_de[i] != 0)
			u_de_index[j++] = i;            //�洢�������ģ���Ӽ����±꣬���Լ�С������
	}
	for (; j < 3; j++)u_de_index[j] = 0;          //����Ŀռ���0

	float den = 0, num = 0;
	/*����delta_Kp��Kp*/
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * Kp_rule_matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}
	delta_Kp = num / den;
	delta_Kp = Ku_p * delta_Kp;
	if (delta_Kp >= delta_Kp_max)   delta_Kp = delta_Kp_max;
	else if (delta_Kp <= -delta_Kp_max) delta_Kp = -delta_Kp_max;
	Kp += delta_Kp;
	if (Kp < 0)Kp = 0;
	/*����delta_Ki��Ki*/
	den = 0; num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * Ki_rule_matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}

	delta_Ki = num / den;
	delta_Ki = Ku_i * delta_Ki;
	if (delta_Ki >= delta_Ki_max)   delta_Ki = delta_Ki_max;
	else if (delta_Ki <= -delta_Ki_max)  delta_Ki = -delta_Ki_max;
	Ki += delta_Ki;
	if (Ki < 0)Ki = 0;
	/*����delta_Kd��Kd*/
	den = 0; num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * Kd_rule_matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}
	delta_Kd = num / den;
	delta_Kd = Ku_d * delta_Kd;
	if (delta_Kd >= delta_Kd_max)   delta_Kd = delta_Kd_max;
	else if (delta_Kd <= -delta_Kd_max) delta_Kd = -delta_Kd_max;
	Kd += delta_Kd;
	if (Kd < 0)Kd = 0;

	A = Kp + Ki + Kd;
	B = -2 * Kd - Kp;
	C = Kd;
	delta_u = A * e + B * e_pre_1 + C * e_pre_2;

	delta_u = delta_u / Ke;

	if (delta_u >= 0.95 * target)delta_u = 0.95 * target;
	else if (delta_u <= -0.95 * target)delta_u = -0.95 * target;

	e_pre_2 = e_pre_1;
	e_pre_1 = e;

	return delta_u;
}
void FuzzyPID::showMf(const string& type, float* mf_paras)
{
	int tab;
	if (type == "trimf")
		tab = 2;
	else if (type == "gaussmf")
		tab == 1;
	else if (type == "trapmf")
		tab = 3;
	cout << "�������ͣ�" << mf_t_e << endl;
	cout << "���������б�" << endl;
	float* p = mf_paras;
	for (int i = 0; i < N * (tab + 1); i++)
	{
		cout.width(3);
		cout << p[i] << "  ";
		if (i % (tab + 1) == tab)
			cout << endl;
	}
}
void FuzzyPID::showInfo()
{
	cout << "Info of this fuzzy controller is as following:" << endl;
	cout << "��������e��[" << -emax << "," << emax << "]" << endl;
	cout << "��������de��[" << -demax << "," << demax << "]" << endl;
	cout << "��������delta_Kp��[" << -delta_Kp_max << "," << delta_Kp_max << "]" << endl;
	cout << "��������delta_Ki��[" << -delta_Ki_max << "," << delta_Ki_max << "]" << endl;
	cout << "��������delta_Kd��[" << -delta_Kd_max << "," << delta_Kd_max << "]" << endl;
	cout << "���e��ģ�������Ⱥ���������" << endl;
	showMf(mf_t_e, e_mf_paras);
	cout << "���仯��de��ģ�������Ⱥ���������" << endl;
	showMf(mf_t_de, de_mf_paras);
	cout << "delta_Kp��ģ�������Ⱥ���������" << endl;
	showMf(mf_t_Kp, Kp_mf_paras);
	cout << "delta_Ki��ģ�������Ⱥ���������" << endl;
	showMf(mf_t_Ki, Ki_mf_paras);
	cout << "delta_Kd��ģ�������Ⱥ���������" << endl;
	showMf(mf_t_Kd, Kd_mf_paras);
	cout << "ģ�������" << endl;
	cout << "delta_Kp��ģ���������" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout.width(3);
			cout << Kp_rule_matrix[i][j] << "  ";
		}
		cout << endl;
	}
	cout << "delta_Ki��ģ���������" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout.width(3);
			cout << Ki_rule_matrix[i][j] << "  ";
		}
		cout << endl;
	}
	cout << "delta_Kd��ģ���������" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout.width(3);
			cout << Kd_rule_matrix[i][j] << "  ";
		}
		cout << endl;
	}
	cout << endl;
	cout << "����������������Ke=" << Ke << endl;
	cout << "���仯�ʵ�������������Kde=" << Kde << endl;
	cout << "�����������������Ku_p=" << Ku_p << endl;
	cout << "�����������������Ku_i=" << Ku_i << endl;
	cout << "�����������������Ku_d=" << Ku_d << endl;
	cout << "�趨Ŀ��target=" << target << endl;
	cout << "���e=" << e << endl;
	cout << "Kp=" << Kp << endl;
	cout << "Ki=" << Ki << endl;
	cout << "Kd=" << Kd << endl;
	cout << endl;
}