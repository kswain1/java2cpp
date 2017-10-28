#include <stdio.h>
#include "csvReader.h"
#include "quaternion.h"

typedef struct sIMURec
{
	float t;
	float ax;
	float ay;
	float az;
	float wx;
	float wy;
	float wz;
}sIMURec;

int main(int argc, char **argv)
{
	csvReader *R;
	if(argc < 2)
	{
		printf("no input file specified - assuming ../accel_mag.csv \n");
		R = new csvReader("../accel_mag.csv");
	}
	else
		R = new csvReader(argv[1]);
	int cnt = R->getLinesCount();
	sIMURec *recs = new sIMURec[cnt];
	for(int x = 0; x < cnt; x++)
	{
		recs[x].t = R->readDouble();
		recs[x].ax = R->readDouble();
		recs[x].ay = R->readDouble();
		recs[x].az = R->readDouble();
		recs[x].wx = R->readDouble();
		recs[x].wy = R->readDouble();
		recs[x].wz = R->readDouble();
	}
	delete R;
	
	sQ Qsg; //quaternion sensor -> ground, need to calculate from initial ground vector
	Qsg.w = 1;
	Qsg.x = 0;
	Qsg.y = 0;
	Qsg.z = 0;
	
	sV cur_speed;
	cur_speed.x = 0;
	cur_speed.y = 0;
	cur_speed.z = 0;

	sV cur_position;
	cur_position.x = 0;
	cur_position.y = 0;
	cur_position.z = 0;
	
	float d2r = M_PI / 180.0;
	int g_timeout = 100;
	int g_low_time = g_timeout;
	for(int x = 1; x < cnt-50; x++)
	{
		float dt = recs[x].t - recs[x-1].t;
		
		float gg = recs[x+30].ax*recs[x+30].ax + recs[x+30].ay*recs[x+30].ay + recs[x+30].az*recs[x+30].az;
		gg = sqrt(gg);

		if(gg >= 5.0)
			g_low_time = 0;
		
		if(gg < 5.0)
		{
			g_low_time++;
			if(g_low_time > g_timeout)
			{
				cur_speed.x = 0;
				cur_speed.y = 0;
				cur_speed.z = 0;

				cur_position.x = 0;
				cur_position.y = 0;
				cur_position.z = 0;
			}
		}
		
		sQ dQsg_x; //rotating Q around x axis
		sQ dQsg_y; //rotating Q around y axis 
		sQ dQsg_z; //rotating Q around z axis
		
		float wx = d2r * recs[x].wx;
		float wy = d2r * recs[x].wy;
		float wz = d2r * recs[x].wz;
		dQsg_x.w = cos(0.5*wx*dt); //quatertion = [cos(a/2), sin(a/2)*rotation_axis]
		dQsg_x.x = sin(0.5*wx*dt);
		dQsg_x.y = 0;
		dQsg_x.z = 0;
		
		dQsg_y.w = cos(0.5*wy*dt);
		dQsg_y.x = 0;
		dQsg_y.y = sin(0.5*wy*dt);
		dQsg_y.z = 0;
		
		dQsg_z.w = cos(0.5*wz*dt);
		dQsg_z.x = 0;
		dQsg_z.y = 0;
		dQsg_z.z = sin(0.5*wz*dt);
		
		Qsg = q_mult(&Qsg, &dQsg_x);
		Qsg = q_mult(&Qsg, &dQsg_y);
		Qsg = q_mult(&Qsg, &dQsg_z);
		q_renorm(&Qsg);
		
		sV accel;
		accel.x = recs[x].ax;
		accel.y = recs[x].ay;
		accel.z = recs[x].az;
		rotate_v(&Qsg, &accel);
//		accel.y += 1.0; //compensate g
		accel.z += 1.0; //compensate g
		
//		accel.z -= 1.0; //compensate g
		float g0 = 9.81;
		
		cur_speed.x += accel.x*dt*g0;
		cur_speed.y += accel.y*dt*g0;
		cur_speed.z += accel.z*dt*g0;
		
		cur_position.x += cur_speed.x*dt;
		cur_position.y += cur_speed.y*dt;
		cur_position.z += cur_speed.z*dt;
		
		if(g_low_time < g_timeout)
		{
			printf("t: %g, A %g (%g %g %g) ", recs[x].t, v_norm(&accel), accel.x, accel.y, accel.z);
			printf("V %g (%g %g %g) ", v_norm(&cur_speed), cur_speed.x, cur_speed.y, cur_speed.z);
			printf("Px %g Py %g Pz %g\n", cur_position.x, cur_position.y, cur_position.z);
		}
		else
		{
			if(x%100 == 101) //never happens, was used for debug
			{
				sV sx, sy, sz;
				sx.x = 1;
				sx.y = 0;
				sx.z = 0;
				sy.x = 0;
				sy.y = 1;
				sy.z = 0;
				sz.x = 0;
				sz.y = 0;
				sz.z = 1;
				rotate_v(&Qsg, &sx);
				rotate_v(&Qsg, &sy);
				rotate_v(&Qsg, &sz);

				printf("t: %g, S -> gnd:\n", recs[x].t); 
				printf("%g %g %g\n", sx.x, sx.y, sx.z);
				printf("%g %g %g\n", sy.x, sy.y, sy.z);
				printf("%g %g %g\n", sz.x, sz.y, sz.z);
				printf("accel: %g %g %g\n", accel.x, accel.y, accel.z);
			}
		}
	}
	
	return 0;
}
