///////////////////////
// Numerical Recipes //
///////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include "nrutil.h"


#define NR_END 1
#define FREE_ARG char*


#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------

void nrerror(char error_text[])
/* Numerical Recipes standard error handler */
{
    printf("\nNumerical Recipes run-time error...\n");
    printf("%s\n",error_text);
}

//---------------------------------------

float *vector(unsigned int nl, unsigned int nh)
/* allocate a float vector with subscript range v[nl..nh] */
{
    float *v;

    v=(float *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(float)));
    if (!v) nrerror("allocation failure in vector()");
    return v-nl+NR_END;
}

//---------------------------------------

int *ivector(unsigned int nl, unsigned int nh)
/* allocate an int vector with subscript range v[nl..nh] */
{
    int *v;

    v=(int *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(int)));
    if (!v) nrerror("allocation failure in ivector()");
    return v-nl+NR_END;
}

//---------------------------------------

unsigned char *cvector(long nl, long nh)
/* allocate an unsigned char vector with subscript range v[nl..nh] */
{
    unsigned char *v;

    v=(unsigned char *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(unsigned char)));
    if (!v) nrerror("allocation failure in cvector()");
    return v-nl+NR_END;
}

//---------------------------------------

unsigned long *lvector(long nl, long nh)
/* allocate an unsigned long vector with subscript range v[nl..nh] */
{
    unsigned long *v;

    v=(unsigned long *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(long)));
    if (!v) nrerror("allocation failure in lvector()");
    return v-nl+NR_END;
}

//---------------------------------------

double *dvector(long nl, long nh)
/* allocate a double vector with subscript range v[nl..nh] */
{
    double *v;

    v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
    if (!v) nrerror("allocation failure in dvector()");
    return v-nl+NR_END;
}

//---------------------------------------

float **matrix(unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch)
/* allocate a float matrix with subscript range m[nrl..nrh][ncl..nch] */
{
    unsigned int i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
    float **m;

    /* allocate pointers to rows */
    m=(float **) malloc((size_t)((nrow+NR_END)*sizeof(float*)));
    if (!m) nrerror("allocation failure 1 in matrix()");
    m += NR_END;
    m -= nrl;

    /* allocate rows and set pointers to them */
    m[nrl]=(float *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float)));
    if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
    m[nrl] += NR_END;
    m[nrl] -= ncl;

    for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

    /* return pointer to array of pointers to rows */
    return m;
}

//---------------------------------------

double **dmatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
    long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
    double **m;

    /* allocate pointers to rows */
    m=(double **) malloc((size_t)((nrow+NR_END)*sizeof(double*)));
    if (!m) nrerror("allocation failure 1 in matrix()");
    m += NR_END;
    m -= nrl;

    /* allocate rows and set pointers to them */
    m[nrl]=(double *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
    if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
    m[nrl] += NR_END;
    m[nrl] -= ncl;

    for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

    /* return pointer to array of pointers to rows */
    return m;
}

//---------------------------------------

int **imatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a int matrix with subscript range m[nrl..nrh][ncl..nch] */
{
    long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
    int **m;

    /* allocate pointers to rows */
    m=(int **) malloc((size_t)((nrow+NR_END)*sizeof(int*)));
    if (!m) nrerror("allocation failure 1 in matrix()");
    m += NR_END;
    m -= nrl;


    /* allocate rows and set pointers to them */
    m[nrl]=(int *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(int)));
    if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
    m[nrl] += NR_END;
    m[nrl] -= ncl;

    for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

    /* return pointer to array of pointers to rows */
    return m;
}

//---------------------------------------

float **submatrix(float **a, long oldrl, long oldrh, long oldcl, long oldch,
    long newrl, long newcl)
/* point a submatrix [newrl..][newcl..] to a[oldrl..oldrh][oldcl..oldch] */
{
    long i,j,nrow=oldrh-oldrl+1,ncol=oldcl-newcl;
    float **m;

    /* allocate array of pointers to rows */
    m=(float **) malloc((size_t) ((nrow+NR_END)*sizeof(float*)));
    if (!m) nrerror("allocation failure in submatrix()");
    m += NR_END;
    m -= newrl;

    /* set pointers to rows */
    for(i=oldrl,j=newrl;i<=oldrh;i++,j++) m[j]=a[i]+ncol;

    /* return pointer to array of pointers to rows */
    return m;
}

//---------------------------------------

float **convert_matrix(float *a, long nrl, long nrh, long ncl, long nch)
/* allocate a float matrix m[nrl..nrh][ncl..nch] that points to the matrix
declared in the standard C manner as a[nrow][ncol], where nrow=nrh-nrl+1
and ncol=nch-ncl+1. The routine should be called with the address
&a[0][0] as the first argument. */
{
    long i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1;
    float **m;

    /* allocate pointers to rows */
    m=(float **) malloc((size_t) ((nrow+NR_END)*sizeof(float*)));
    if (!m) nrerror("allocation failure in convert_matrix()");
    m += NR_END;
    m -= nrl;

    /* set pointers to rows */
    m[nrl]=a-ncl;
    for(i=1,j=nrl+1;i<nrow;i++,j++) m[j]=m[j-1]+ncol;
    /* return pointer to array of pointers to rows */
    return m;
}

//---------------------------------------

float ***f3tensor(long nrl, long nrh, long ncl, long nch, long ndl, long ndh)
/* allocate a float 3tensor with range t[nrl..nrh][ncl..nch][ndl..ndh] */
{
    long i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1,ndep=ndh-ndl+1;
    float ***t;

    /* allocate pointers to pointers to rows */
    t=(float ***) malloc((size_t)((nrow+NR_END)*sizeof(float**)));
    if (!t) nrerror("allocation failure 1 in f3tensor()");
    t += NR_END;
    t -= nrl;

    /* allocate pointers to rows and set pointers to them */
    t[nrl]=(float **) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float*)));
    if (!t[nrl]) nrerror("allocation failure 2 in f3tensor()");
    t[nrl] += NR_END;
    t[nrl] -= ncl;

    /* allocate rows and set pointers to them */
    t[nrl][ncl]=(float *) malloc((size_t)((nrow*ncol*ndep+NR_END)*sizeof(float)));
    if (!t[nrl][ncl]) nrerror("allocation failure 3 in f3tensor()");
    t[nrl][ncl] += NR_END;
    t[nrl][ncl] -= ndl;

    for(j=ncl+1;j<=nch;j++) t[nrl][j]=t[nrl][j-1]+ndep;
    for(i=nrl+1;i<=nrh;i++) {
        t[i]=t[i-1]+ncol;
        t[i][ncl]=t[i-1][ncl]+ncol*ndep;
        for(j=ncl+1;j<=nch;j++) t[i][j]=t[i][j-1]+ndep;
    }

    /* return pointer to array of pointers to rows */
    return t;
}

//---------------------------------------

void free_vector(float *v, unsigned int nl, unsigned int nh)
/* free a float vector allocated with vector() */
{
    free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_ivector(int *v, unsigned int nl, unsigned int nh)
/* free an int vector allocated with ivector() */
{
    free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_cvector(unsigned char *v, long nl, long nh)
/* free an unsigned char vector allocated with cvector() */
{
    free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_lvector(unsigned long *v, long nl, long nh)
/* free an unsigned long vector allocated with lvector() */
{
    free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_dvector(double *v, long nl, long nh)
/* free a double vector allocated with dvector() */
{
    free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_matrix(float **m, unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch)
/* free a float matrix allocated by matrix() */
{
    free((FREE_ARG) (m[nrl]+ncl-NR_END));
    free((FREE_ARG) (m+nrl-NR_END));
}

//---------------------------------------

void free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch)
/* free a double matrix allocated by dmatrix() */
{
    free((FREE_ARG) (m[nrl]+ncl-NR_END));
    free((FREE_ARG) (m+nrl-NR_END));
}

//---------------------------------------

void free_imatrix(int **m, long nrl, long nrh, long ncl, long nch)
/* free an int matrix allocated by imatrix() */
{
    free((FREE_ARG) (m[nrl]+ncl-NR_END));
    free((FREE_ARG) (m+nrl-NR_END));
}

//---------------------------------------

void free_submatrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a submatrix allocated by submatrix() */
{
    free((FREE_ARG) (b+nrl-NR_END));
}

//---------------------------------------

void free_convert_matrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a matrix allocated by convert_matrix() */
{
    free((FREE_ARG) (b+nrl-NR_END));
}

//---------------------------------------

void free_f3tensor(float ***t, long nrl, long nrh, long ncl, long nch,
    long ndl, long ndh)
/* free a float f3tensor allocated by f3tensor() */
{
    free((FREE_ARG) (t[nrl][ncl]+ndl-NR_END));
    free((FREE_ARG) (t[nrl]+ncl-NR_END));
    free((FREE_ARG) (t+nrl-NR_END));
}

//---------------------------------------

void simp1(double **a, int mm, int ll[], int nll, int iabf, int *kp, double *bmax)
{
    int k;
    double test;

    if (nll <= 0)
        *bmax=0.0;
    else {
        *kp=ll[1];
        *bmax=a[mm+1][*kp+1];
        for (k=2;k<=nll;k++) {
            if (iabf == 0)
                test=a[mm+1][ll[k]+1]-(*bmax);
            else
                test = (double)(fabs(a[mm+1][ll[k]+1])-fabs(*bmax));
            if (test > 0.) {
                *bmax=a[mm+1][ll[k]+1];
                *kp=ll[k];
            }
        }
    }
}

//---------------------------------------

void simp2(double **a, int m, int n, int *ip, int kp)
{
    int k,i;
    double qp,q0,q,q1;

    *ip=0;
    for (i=1;i<=m;i++)
        if (a[i+1][kp+1] < -EPS) break;
    if (i>m) return;
    q1 = -a[i+1][1]/a[i+1][kp+1];
    *ip=i;
    for (i=*ip+1;i<=m;i++) {
        if (a[i+1][kp+1] < -EPS) {
            q = -a[i+1][1]/a[i+1][kp+1];
            if (q < q1) {
                *ip=i;
                q1=q;
            } else if (q == q1) {
                for (k=1;k<=n;k++) {
                    qp = -a[*ip+1][k+1]/a[*ip+1][kp+1];
                    q0 = -a[i+1][k+1]/a[i+1][kp+1];
                    if (q0 != qp) break;
                }
                if (q0 < qp) *ip=i;
            }
        }
    }
}

//---------------------------------------

void simp3(double **a, int i1, int k1, int ip, int kp)
{
    int kk,ii;
    double piv;

    piv=1.0f/a[ip+1][kp+1];
    for (ii=1;ii<=i1+1;ii++)
        if (ii-1 != ip) {
            a[ii][kp+1] *= piv;
            for (kk=1;kk<=k1+1;kk++)
                if (kk-1 != kp)
                    a[ii][kk] -= a[ip+1][kk]*a[ii][kp+1];
        }
    for (kk=1;kk<=k1+1;kk++)
        if (kk-1 != kp) a[ip+1][kk] *= -piv;
    a[ip+1][kp+1]=piv;
}

//---------------------------------------

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int gaussj(float **A, int n, float *X)
{
    int i,icol,irow,j,k,l,ll;
    float big,dum,pivinv,temp;
    int *indxc,*indxr,*ipiv;

    indxc=ivector(1,n);
    indxr=ivector(1,n);
    ipiv=ivector(1,n);
    for (j=1;j<=n;j++)
        ipiv[j]=0;

    for (i=1;i<=n;i++)
    {
        big=0.;
        for (j=1;j<=n;j++)
        {
            if (ipiv[j] != 1)
            {
                for (k=1;k<=n;k++)
                {
                    if (ipiv[k] == 0)
                    {
                        if (fabs(A[j][k]) >= big)
                        {
                            big=(float)fabs(A[j][k]);
                            irow=j;
                            icol=k;
                        }
                    }
                }
            }
        }
        ++(ipiv[icol]);
        if (irow != icol) {
            for (l=1;l<=n;l++)
                SWAP(A[irow][l],A[icol][l])
            SWAP(X[irow],X[icol])
        }
        indxr[i]=irow;
        indxc[i]=icol;
        if (A[icol][icol] == 0.0)
            return 1;
        pivinv=1.f/A[icol][icol];
        A[icol][icol]=1.f;
        for (l=1;l<=n;l++)
            A[icol][l] *= pivinv;
        X[icol] *= pivinv;
        for (ll=1;ll<=n;ll++)
            if (ll != icol) {
                dum=A[ll][icol];
                A[ll][icol]=0.0;
                for (l=1;l<=n;l++)
                    A[ll][l] -= A[icol][l]*dum;
                X[ll] -= X[icol]*dum;
            }
    }

    for (l=n;l>=1;l--) {
        if (indxr[l] != indxc[l])
            for (k=1;k<=n;k++)
                SWAP(A[k][indxr[l]],A[k][indxc[l]]);
    }
    free_ivector(ipiv,1,n);
    free_ivector(indxr,1,n);
    free_ivector(indxc,1,n);

    return 0;
}
#undef SWAP

//---------------------------------------

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int inv(float **Ai, int n)
{
    int *indxc,*indxr,*ipiv;
    int i,icol,irow,j,k,l,ll;
    float big,dum,pivinv, temp;

    indxc=ivector(1,n);
    indxr=ivector(1,n);
    ipiv=ivector(1,n);
    for (j=1;j<=n;j++)
        ipiv[j]=0;

    for (i=1;i<=n;i++)
    {
        big=0.;
        for (j=1;j<=n;j++)
        {
            if (ipiv[j] != 1)
            {
                for (k=1;k<=n;k++)
                {
                    if (ipiv[k] == 0)
                    {
                        if (fabs(Ai[j][k]) >= big)
                        {
                            big=(float)fabs(Ai[j][k]);
                            irow=j;
                            icol=k;
                        }
                    }
                }
            }
        }
        ++(ipiv[icol]);
        if (irow != icol) {
            for (l=1;l<=n;l++)
                SWAP(Ai[irow][l],Ai[icol][l])
        }
        indxr[i]=irow;
        indxc[i]=icol;
        if (fabs(Ai[icol][icol]) <= EPS)
            return -1;
        pivinv=1.f/Ai[icol][icol];
        Ai[icol][icol]=1.f;
        for (l=1;l<=n;l++)
            Ai[icol][l] *= pivinv;
        for (ll=1;ll<=n;ll++)
            if (ll != icol) {
                dum=Ai[ll][icol];
                Ai[ll][icol]=0.0;
                for (l=1;l<=n;l++)
                    Ai[ll][l] -= Ai[icol][l]*dum;
            }
    }

    for (l=n;l>=1;l--) {
        if (indxr[l] != indxc[l])
            for (k=1;k<=n;k++)
                SWAP(Ai[k][indxr[l]],Ai[k][indxc[l]]);
    }
    free_ivector(ipiv,1,n);
    free_ivector(indxr,1,n);
    free_ivector(indxc,1,n);

    return 0;
}
#undef SWAP

//---------------------------------------

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int dinv(double **Ai, int n)  // inv() for double
{
    int *indxc,*indxr,*ipiv;
    int i,icol,irow,j,k,l,ll;
    double big,dum,pivinv, temp;

    indxc=ivector(1,n);
    indxr=ivector(1,n);
    ipiv=ivector(1,n);
    for (j=1;j<=n;j++)
        ipiv[j]=0;

    for (i=1;i<=n;i++)
    {
        big=0.;
        for (j=1;j<=n;j++)
        {
            if (ipiv[j] != 1)
            {
                for (k=1;k<=n;k++)
                {
                    if (ipiv[k] == 0)
                    {
                        if (fabs(Ai[j][k]) >= big)
                        {
                            big=fabs(Ai[j][k]);
                            irow=j;
                            icol=k;
                        }
                    }
                }
            }
        }
        ++(ipiv[icol]);
        if (irow != icol) {
            for (l=1;l<=n;l++)
                SWAP(Ai[irow][l],Ai[icol][l]);
        }
        indxr[i]=irow;
        indxc[i]=icol;
        if (Ai[icol][icol] == 0.0)
            return -1;
        pivinv=1.f/Ai[icol][icol];
        Ai[icol][icol]=1.f;
        for (l=1;l<=n;l++)
            Ai[icol][l] *= pivinv;
        for (ll=1;ll<=n;ll++)
            if (ll != icol) {
                dum=Ai[ll][icol];
                Ai[ll][icol]=0.0;
                for (l=1;l<=n;l++)
                    Ai[ll][l] -= Ai[icol][l]*dum;
            }
    }

    for (l=n;l>=1;l--) {
        if (indxr[l] != indxc[l])
            for (k=1;k<=n;k++)
                SWAP(Ai[k][indxr[l]],Ai[k][indxc[l]]);
    }
    free_ivector(ipiv,1,n);
    free_ivector(indxr,1,n);
    free_ivector(indxc,1,n);

    return 0;
}
#undef SWAP

//---------------------------------------

int svdcmp(float **a, int m, int n, float w[], float **v)
{
    int flag,i,its,j,jj,k,l,nm;
    float anorm,c,f,g,h,s,scale,x,y,z;
    float *rv1;

    rv1=vector(1,n);
    g=scale=anorm=0.0;
    for (i=1;i<=n;i++)
    {
        l=i+1;
        rv1[i]=scale*g;
        g=s=scale=0.0;
        if (i <= m)
        {
            for (k=i;k<=m;k++)
                scale += (float)fabs(a[k][i]);
            if (scale)
            {
                for (k=i;k<=m;k++)
                {
                    a[k][i] /= scale;
                    s += a[k][i]*a[k][i];
                }
                f=a[i][i];
                g = -(float)SIGN(sqrt(s),f);
                h=f*g-s;
                a[i][i]=f-g;
                for (j=l;j<=n;j++)
                {
                    for (s=0.0,k=i;k<=m;k++)
                        s += a[k][i]*a[k][j];
                    f=s/h;
                    for (k=i;k<=m;k++)
                        a[k][j] += f*a[k][i];
                }
                for (k=i;k<=m;k++)
                    a[k][i] *= scale;
            }
        }
        w[i]=scale *g;
        g=s=scale=0.0;
        if (i <= m && i != n)
        {
            for (k=l;k<=n;k++)
                scale += (float)fabs(a[i][k]);
            if (scale)
            {
                for (k=l;k<=n;k++)
                {
                    a[i][k] /= scale;
                    s += a[i][k]*a[i][k];
                }
                f=a[i][l];
                g = -(float)SIGN(sqrt(s),f);
                h=f*g-s;
                a[i][l]=f-g;
                for (k=l;k<=n;k++)
                    rv1[k]=a[i][k]/h;
                for (j=l;j<=m;j++)
                {
                    for (s=0.0,k=l;k<=n;k++)
                        s += a[j][k]*a[i][k];
                    for (k=l;k<=n;k++)
                        a[j][k] += s*(float)rv1[k];
                }
                for (k=l;k<=n;k++)
                    a[i][k] *= scale;
            }
        }
        anorm = (float)FMAX(anorm,(float)(fabs(w[i])+fabs(rv1[i])));
    }
    for (i=n;i>=1;i--)
    {
        if (i < n)
        {
            if (g)
            {
                for (j=l;j<=n;j++)
                    v[j][i]=(a[i][j]/a[i][l])/g;
                for (j=l;j<=n;j++)
                {
                    for (s=0.0,k=l;k<=n;k++)
                        s += a[i][k]*v[k][j];
                    for (k=l;k<=n;k++)
                        v[k][j] += s*v[k][i];
                }
            }
            for (j=l;j<=n;j++)
                v[i][j]=v[j][i]=0.0;
        }
        v[i][i]=1.0;
        g=(float)rv1[i];
        l=i;
    }
    for (i=IMIN(m,n);i>=1;i--)
    {
        l=i+1;
        g=w[i];
        for (j=l;j<=n;j++)
            a[i][j]=0.0;
        if (g) {
            g=1.f/g;
            for (j=l;j<=n;j++)
            {
                for (s=0.0,k=l;k<=m;k++)
                    s += a[k][i]*a[k][j];
                f=(s/a[i][i])*g;
                for (k=i;k<=m;k++)
                    a[k][j] += f*a[k][i];
            }
            for (j=i;j<=m;j++)
                a[j][i] *= g;
        }
        else
            for (j=i;j<=m;j++) a[j][i]=0.0;
        ++a[i][i];
    }
    for (k=n;k>=1;k--)
    {
        for (its=1;its<=30;its++)
        {
            flag=1;
            for (l=k;l>=1;l--)
            {
                nm=l-1;
                //if ((float)(fabs(rv1[l])+anorm) == anorm)
                if (fabs(rv1[l]) <= EPS*0.5f)
                {
                    flag=0;
                    break;
                }
                //if ((float)(fabs(w[nm])+anorm) == anorm)
                if (fabs(w[nm])<= EPS*0.5f)
                    break;
            }
            if (flag)
            {
                c=0.0;
                s=1.0;
                for (i=l;i<=k;i++)
                {
                    f=s*(float)rv1[i];
                    rv1[i]=c*rv1[i];
                    //if ((float)(fabs(f)+anorm) == anorm)
                    if (fabs(f)<= EPS*0.5f)
                        break;
                    g=w[i];
                    h=pythag(f,g);
                    w[i]=h;
                    h=1.f/h;
                    c=g*h;
                    s = -f*h;
                    for (j=1;j<=m;j++)
                    {
                        y=a[j][nm];
                        z=a[j][i];
                        a[j][nm]=y*c+z*s;
                        a[j][i]=z*c-y*s;
                    }
                }
            }
            z=w[k];
            if (l == k)
            {
                if (z < 0.0)
                {
                    w[k] = -z;
                    for (j=1;j<=n;j++)
                        v[j][k] = -v[j][k];
                }
                break;
            }
            if (its == 30)
            {
                nrerror("no convergence in 30 svdcmp iterations");
                return -1;
            }
            x=w[l];
            nm=k-1;
            y=w[nm];
            g=(float)rv1[nm];
            h=(float)rv1[k];
            f=((y-z)*(y+z)+(g-h)*(g+h))/(2.f*h*y);
            g=pythag(f,1.f);
            f=((x-z)*(x+z)+h*((y/(f+(float)SIGN(g,f)))-h))/x;
            c=s=1.f;
            for (j=l;j<=nm;j++)
            {
                i=j+1;
                g=(float)rv1[i];
                y=w[i];
                h=s*g;
                g=c*g;
                z=pythag(f,h);
                rv1[j]=z;
                c=f/z;
                s=h/z;
                f=x*c+g*s;
                g = g*c-x*s;
                h=y*s;
                y *= c;
                for (jj=1;jj<=n;jj++)
                {
                    x=v[jj][j];
                    z=v[jj][i];
                    v[jj][j]=x*c+z*s;
                    v[jj][i]=z*c-x*s;
                }
                z=pythag(f,h);
                w[j]=z;
                if (z)
                {
                    z=1.f/z;
                    c=f*z;
                    s=h*z;
                }
                f=c*g+s*y;
                x=c*y-s*g;
                for (jj=1;jj<=m;jj++)
                {
                    y=a[jj][j];
                    z=a[jj][i];
                    a[jj][j]=y*c+z*s;
                    a[jj][i]=z*c-y*s;
                }
            }
            rv1[l]=0.0;
            rv1[k]=f;
            w[k]=x;
        }
    }

    free_vector(rv1,1,n);
    return 0;
}

//---------------------------------------

int dsvdcmp(double **a, int m, int n, double w[], double **v)  //svdcmp() for double
{
    int flag,i,its,j,jj,k,l,nm;
    double anorm,c,f,g,h,s,scale,x,y,z;
    double *rv1;

    rv1=dvector(1,n);
    g=scale=anorm=0.0;
    for (i=1;i<=n;i++)
    {
        l=i+1;
        rv1[i]=scale*g;
        g=s=scale=0.0;
        if (i <= m)
        {
            for (k=i;k<=m;k++)
                scale += fabs(a[k][i]);
            if (scale)
            {
                for (k=i;k<=m;k++)
                {
                    a[k][i] /= scale;
                    s += a[k][i]*a[k][i];
                }
                f=a[i][i];
                g = -SIGN(sqrt(s),f);
                h=f*g-s;
                a[i][i]=f-g;
                for (j=l;j<=n;j++)
                {
                    for (s=0.0,k=i;k<=m;k++)
                        s += a[k][i]*a[k][j];
                    f=s/h;
                    for (k=i;k<=m;k++)
                        a[k][j] += f*a[k][i];
                }
                for (k=i;k<=m;k++)
                    a[k][i] *= scale;
            }
        }
        w[i]=scale *g;
        g=s=scale=0.0;
        if (i <= m && i != n)
        {
            for (k=l;k<=n;k++)
                scale += fabs(a[i][k]);
            if (scale)
            {
                for (k=l;k<=n;k++)
                {
                    a[i][k] /= scale;
                    s += a[i][k]*a[i][k];
                }
                f=a[i][l];
                g = -SIGN(sqrt(s),f);
                h=f*g-s;
                a[i][l]=f-g;
                for (k=l;k<=n;k++)
                    rv1[k]=a[i][k]/h;
                for (j=l;j<=m;j++)
                {
                    for (s=0.0,k=l;k<=n;k++)
                        s += a[j][k]*a[i][k];
                    for (k=l;k<=n;k++)
                        a[j][k] += s*rv1[k];
                }
                for (k=l;k<=n;k++)
                    a[i][k] *= scale;
            }
        }
        anorm = DMAX(anorm,(fabs(w[i])+fabs(rv1[i])));
    }
    for (i=n;i>=1;i--)
    {
        if (i < n)
        {
            if (g)
            {
                for (j=l;j<=n;j++)
                    v[j][i]=(a[i][j]/a[i][l])/g;
                for (j=l;j<=n;j++)
                {
                    for (s=0.0,k=l;k<=n;k++)
                        s += a[i][k]*v[k][j];
                    for (k=l;k<=n;k++)
                        v[k][j] += s*v[k][i];
                }
            }
            for (j=l;j<=n;j++)
                v[i][j]=v[j][i]=0.0;
        }
        v[i][i]=1.0;
        g=rv1[i];
        l=i;
    }
    for (i=IMIN(m,n);i>=1;i--)
    {
        l=i+1;
        g=w[i];
        for (j=l;j<=n;j++)
            a[i][j]=0.0;
        if (g) {
            g=1.f/g;
            for (j=l;j<=n;j++)
            {
                for (s=0.0,k=l;k<=m;k++)
                    s += a[k][i]*a[k][j];
                f=(s/a[i][i])*g;
                for (k=i;k<=m;k++)
                    a[k][j] += f*a[k][i];
            }
            for (j=i;j<=m;j++)
                a[j][i] *= g;
        }
        else
            for (j=i;j<=m;j++) a[j][i]=0.0;
        ++a[i][i];
    }
    for (k=n;k>=1;k--)
    {
        for (its=1;its<=30;its++)
        {
            flag=1;
            for (l=k;l>=1;l--)
            {
                nm=l-1;
                //if ((float)(fabs(rv1[l])+anorm) == anorm)
                if (fabs(rv1[l]) <= EPS*0.5f)
                {
                    flag=0;
                    break;
                }
                //if ((float)(fabs(w[nm])+anorm) == anorm)
                if (fabs(w[nm])<= EPS*0.5f)
                    break;
            }
            if (flag)
            {
                c=0.0;
                s=1.0;
                for (i=l;i<=k;i++)
                {
                    f=s*rv1[i];
                    rv1[i]=c*rv1[i];
                    //if ((float)(fabs(f)+anorm) == anorm)
                    if (fabs(f)<= EPS*0.5f)
                        break;
                    g=w[i];
                    h=dpythag(f,g);
                    w[i]=h;
                    h=1.f/h;
                    c=g*h;
                    s = -f*h;
                    for (j=1;j<=m;j++)
                    {
                        y=a[j][nm];
                        z=a[j][i];
                        a[j][nm]=y*c+z*s;
                        a[j][i]=z*c-y*s;
                    }
                }
            }
            z=w[k];
            if (l == k)
            {
                if (z < 0.0)
                {
                    w[k] = -z;
                    for (j=1;j<=n;j++)
                        v[j][k] = -v[j][k];
                }
                break;
            }
            if (its == 30)
            {
                nrerror("no convergence in 30 svdcmp iterations");
                return -1;
            }
            x=w[l];
            nm=k-1;
            y=w[nm];
            g=rv1[nm];
            h=rv1[k];
            f=((y-z)*(y+z)+(g-h)*(g+h))/(2.f*h*y);
            g=dpythag(f,1.f);
            f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
            c=s=1.f;
            for (j=l;j<=nm;j++)
            {
                i=j+1;
                g=rv1[i];
                y=w[i];
                h=s*g;
                g=c*g;
                z=dpythag(f,h);
                rv1[j]=z;
                c=f/z;
                s=h/z;
                f=x*c+g*s;
                g = g*c-x*s;
                h=y*s;
                y *= c;
                for (jj=1;jj<=n;jj++)
                {
                    x=v[jj][j];
                    z=v[jj][i];
                    v[jj][j]=x*c+z*s;
                    v[jj][i]=z*c-x*s;
                }
                z=dpythag(f,h);
                w[j]=z;
                if (z)
                {
                    z=1.f/z;
                    c=f*z;
                    s=h*z;
                }
                f=c*g+s*y;
                x=c*y-s*g;
                for (jj=1;jj<=m;jj++)
                {
                    y=a[jj][j];
                    z=a[jj][i];
                    a[jj][j]=y*c+z*s;
                    a[jj][i]=z*c-y*s;
                }
            }
            rv1[l]=0.0;
            rv1[k]=f;
            w[k]=x;
        }
    }

    free_dvector(rv1,1,n);
    return 0;
}

//---------------------------------------

float pythag(float a, float b)
{
    float absa,absb;
    absa = (float)fabs(a);
    absb = (float)fabs(b);
    if (absa > absb)
        return (float)(absa*sqrt(1.0+SQR(absb/absa)));
    else
        return (float)(absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}

//---------------------------------------

double dpythag(double a, double b)
{
    double absa,absb;
    absa = fabs(a);
    absb = fabs(b);
    if (absa > absb)
        return (absa*sqrt(1.0+DSQR(absb/absa)));
    else
        return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+DSQR(absa/absb)));
}

//---------------------------------------

#define SWAP(a,b) temp=(a);(a)=(b);(b)=temp;
#define SWAPi(a,b) tempi=(a);(a)=(b);(b)=tempi;
// example
// arr_4x1[] = {0, 4, 3, 2, 1}
// index_4x1[] = {0, 1, 2, 3, 4}
//
// nrselect(1,4, arr_4x1, index_4x1)
// arr_4x1 : {0, 1, 2, 3, 4}
// index_4x1 : {0, 4, 3, 2, 1}
// returned value : 1

float nrselect(unsigned int k, unsigned int n, float arr[], unsigned int index[])
{
    unsigned int i,ir,j,l,mid;
    float a,temp;
    unsigned int ind, tempi;

    l=1;
    ir=n;
    for (;;) {
        if (ir <= l+1) {
            if (ir == l+1 && arr[ir] < arr[l]) {
                SWAP(arr[l],arr[ir])
                SWAPi(index[l],index[ir])
            }
            return arr[k];
        } else {
            mid=(l+ir) >> 1;
            SWAP(arr[mid],arr[l+1])
            SWAPi(index[mid], index[l+1])
            if (arr[l] > arr[ir]) {
                SWAP(arr[l],arr[ir])
                SWAPi(index[l],index[ir])
            }
            if (arr[l+1] > arr[ir]) {
                SWAP(arr[l+1],arr[ir])
                SWAPi(index[l+1],index[ir])
            }
            if (arr[l] > arr[l+1]) {
                SWAP(arr[l],arr[l+1])
                SWAPi(index[l],index[l+1])
            }
            i=l+1;
            j=ir;
            a=arr[l+1];
            ind = index[l+1];
            for (;;) {
                do i++; while (arr[i] < a);
                do j--; while (arr[j] > a);
                if (j < i) break;
                SWAP(arr[i],arr[j])
                SWAPi(index[i],index[j])
            }
            arr[l+1]=arr[j];
            index[l+1] = index[j];
            arr[j]=a;
            index[j] = ind;
            if (j >= k) ir=j-1;
            if (j <= k) l=i;
        }
    }
}
#undef SWAP
#undef SWAPi


#ifdef __cplusplus
}
#endif
