#include <stdio.h>
#include <iostream>
#include <math.h>
// 함수
void inv_mat12(int m, int n, double Mat12[][12], double c_inv[12][12])   
{
	double big,size,abig, cbig, ratio; 
	int i, k, j, ibig;
	double jcob[12][12];
	
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			if(n==12)
				jcob[i][j]=Mat12[i][j];
		}
	}

	if(n==12) {
	for(i=m;i<n;i++)
	{
		for(j=m;j<n;j++)
			{
			c_inv[i][j]=0.;
			if(i==j) c_inv[i][i]=1.;
			}
	}
	for (k=m; k<n; k++)
		{
		big=fabs(jcob[k][k]);
		ibig=k;

		for (i=k;i<n;i++)
			{
			size=fabs(jcob[i][k]);
			if(size < big) goto next;
			big=size;
			ibig=i;
next:;			
			}
	
		if(k==ibig) goto next2;
		for(j=m;j<n;j++)
			{
			if(j>=k)
				{
				abig=jcob[ibig][j];
				jcob[ibig][j]=jcob[k][j];
				jcob[k][j]=abig;
				}
			cbig=c_inv[ibig][j];
			c_inv[ibig][j]=c_inv[k][j];
			c_inv[k][j]=cbig;
 			}
		



next2:;

		if(jcob[k][k] == 0.) {/*lcd_putch('S'); halt_e();return(1);*/}
		for(i=m;i<n;i++)
			{
			if(i==k) goto next3;
			ratio=jcob[i][k]/jcob[k][k];
			for (j=m;j <n;j++)
				{
				if(j>=k) jcob[i][j]=jcob[i][j] - ratio*jcob[k][j];
				c_inv[i][j]=c_inv[i][j] - ratio*c_inv[k][j];
				}


next3:;

			}
		}
	for (k=m; k<n; k++)
		{
		for(j=m;j<n;j++)
			{
			c_inv[k][j]=c_inv[k][j]/jcob[k][k];
			}
		}
	}
}


// 함수를 사용할 때
int main(void){
	double J[12][12] = {}
	inv_mat12(0,12,J,Inv_J);
	std::cout<<"inv_mat12 : "
	std::cout<<"\t"<<










}











