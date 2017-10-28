#include "quaternion.h"

double q_norm(sQ *q)
{
	return sqrt(q->x*q->x + q->y*q->y + q->z*q->z + q->w*q->w);
}

double v_norm(sV *v)
{
	return sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
}

void q_renorm(sQ *q)
{
	double r = q_norm(q);
	if(r > 0)
	{
		double m = 1.0 / r;
		q->x *= m;	
		q->y *= m;	
		q->z *= m;	
		q->w *= m;	
	}
}

sQ q_getconj(sQ *q)
{
	sQ qc;
	qc.x = -q->x;
	qc.y = -q->y;
	qc.z = -q->z;
	qc.w = q->w;
	return qc;
}

void q_makeconj(sQ *q)
{
	q->x = -q->x;
	q->y = -q->y;
	q->z = -q->z;
}

sQ q_mult(sQ *q1, sQ *q2)
{
// x1  y1  z1
// x2  y2  z2
// x   y   z
	sQ q;
	q.w = q1->w*q2->w - (q1->x*q2->x + q1->y*q2->y + q1->z*q2->z);
	q.x = q1->w*q2->x + q2->w*q1->x + q1->y*q2->z - q1->z*q2->y;
	q.y = q1->w*q2->y + q2->w*q1->y + q1->z*q2->x - q1->x*q2->z;
	q.z = q1->w*q2->z + q2->w*q1->z + q1->x*q2->y - q1->y*q2->x;
	return q;
}

void rotate_v(sQ *q, sV *v)
{
	sQ r;
	r.w = 0;
	r.x = v->x;
	r.y = v->y;
	r.z = v->z;
	sQ qc = q_getconj(q);
	sQ qq = q_mult(&r, &qc);
	sQ rq = q_mult(q, &qq);
	v->x = rq.x;
	v->y = rq.y;
	v->z = rq.z;
}

sQ v_mult(sQ *q1, sQ *q2)
{
// x1  y1  z1
// x2  y2  z2
// x   y   z
	sQ q;
	q.w = 0;
	q.x = q1->y*q2->z - q1->z*q2->y;
	q.y = q1->z*q2->x - q1->x*q2->z;
	q.z = q1->x*q2->y - q1->y*q2->x;
	return q;
}
