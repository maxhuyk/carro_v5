#include "ControlMath.h"
#include <math.h>

// ===== Migración literal de calculos.cpp =====
static float buffers[3][10];
static int buffer_indices[3] = {0,0,0};
static int buffer_counts[3] = {0,0,0};
static int ventana_size = 5;
static int num_sensores = 3;

void media_movil_init(int n_sensores, int ventana){
  num_sensores = n_sensores; ventana_size = ventana;
  for(int i=0;i<n_sensores;i++){ buffer_indices[i]=0; buffer_counts[i]=0; for(int j=0;j<ventana;j++) buffers[i][j]=0.0f; }
}
void media_movil_actualizar(float* nuevos, float* res){
  for(int i=0;i<num_sensores;i++){
    buffers[i][buffer_indices[i]] = nuevos[i];
    buffer_indices[i] = (buffer_indices[i]+1)%ventana_size;
    if(buffer_counts[i] < ventana_size) buffer_counts[i]++;
    float suma=0.0f; for(int j=0;j<buffer_counts[i];j++) suma += buffers[i][j];
    res[i] = suma / buffer_counts[i];
  }
}

static float kalman_x[3]={0,0,0};
static float kalman_P[3]={1,1,1};
static float kalman_Qv=0.01f;
static float kalman_Rv=1.0f;
static int kalman_n=3;

void kalman_init(int n, float Q, float R){ kalman_n=n; kalman_Qv=Q; kalman_Rv=R; for(int i=0;i<n;i++){ kalman_x[i]=0.0f; kalman_P[i]=1.0f; } }
void kalman_filtrar(float* m, float* r){
  for(int i=0;i<kalman_n;i++){
    kalman_P[i] += kalman_Qv;
    float K = kalman_P[i]/(kalman_P[i]+kalman_Rv);
    kalman_x[i] = kalman_x[i] + K*(m[i]-kalman_x[i]);
    kalman_P[i] = (1-K)*kalman_P[i];
    r[i] = kalman_x[i];
  }
}
void kalman_force_state(float* m, float P_init){ for(int i=0;i<kalman_n;i++){ kalman_x[i]=m[i]; kalman_P[i]=P_init; }}
void kalman_set_Q(float Q){ kalman_Qv=Q; }
float kalman_get_Q(){ return kalman_Qv; }

void pid_init(PIDController* pid, float kp,float ki,float kd,float setpoint,float alpha,float salida_max,float integral_max){ pid->kp=kp;pid->ki=ki;pid->kd=kd;pid->setpoint=setpoint;pid->alpha=alpha;pid->error_anterior=0;pid->integral=0;pid->salida_maxima=salida_max;pid->integral_maxima=integral_max; }
float pid_update(PIDController* pid, float medida){ float error = pid->setpoint - medida; float error_f = pid->alpha*error + (1-pid->alpha)*pid->error_anterior; float deriv = error_f - pid->error_anterior; pid->integral += error_f; if(pid->integral_maxima!=0.0f){ if(pid->integral>pid->integral_maxima) pid->integral=pid->integral_maxima; if(pid->integral<-pid->integral_maxima) pid->integral=-pid->integral_maxima; } float salida = pid->kp*error_f + pid->ki*pid->integral + pid->kd*deriv; if(pid->salida_maxima!=0.0f){ if(salida>pid->salida_maxima) salida=pid->salida_maxima; if(salida<-pid->salida_maxima) salida=-pid->salida_maxima; } pid->error_anterior=error_f; return salida; }

void limitar_variacion(float* dist_act, float* dist_ant, float* res, int n, float max_d){ for(int i=0;i<n;i++){ float delta = dist_act[i]-dist_ant[i]; if(delta>max_d) delta=max_d; else if(delta<-max_d) delta=-max_d; res[i]=dist_ant[i]+delta; }}

static float vector_norm(float* v,int sz){ float sum=0; for(int i=0;i<sz;i++) sum+=v[i]*v[i]; return sqrt(sum);} 
static void vector_sub(float* a,float* b,float* r,int sz){ for(int i=0;i<sz;i++) r[i]=a[i]-b[i]; }
static void vector_add(float* a,float* b,float* r,int sz){ for(int i=0;i<sz;i++) r[i]=a[i]+b[i]; }
static void vector_scale(float* v,float s,float* r,int sz){ for(int i=0;i<sz;i++) r[i]=v[i]*s; }
static float vector_dot(float* a,float* b,int sz){ float sum=0; for(int i=0;i<sz;i++) sum+=a[i]*b[i]; return sum; }
static void vector_cross_3d(float* a,float* b,float* r){ r[0]=a[1]*b[2]-a[2]*b[1]; r[1]=a[2]*b[0]-a[0]*b[2]; r[2]=a[0]*b[1]-a[1]*b[0]; }

void trilateracion_3d(const float sp[3][3], float* dist, float* out){
  float P1[3]={sp[0][0],sp[0][1],sp[0][2]}; float P2[3]={sp[1][0],sp[1][1],sp[1][2]}; float P3[3]={sp[2][0],sp[2][1],sp[2][2]};
  float r1=dist[0], r2=dist[1], r3=dist[2];
  float p2_p1[3]; vector_sub(P2,P1,p2_p1,3); float norm_p2_p1 = vector_norm(p2_p1,3); float ex[3]; vector_scale(p2_p1,1.0f/norm_p2_p1,ex,3);
  float ey_dir[3]={0,1,0}; float norm_ey=vector_norm(ey_dir,3); float ey[3]; vector_scale(ey_dir,1.0f/norm_ey,ey,3);
  float ez[3]; vector_cross_3d(ex,ey,ez);
  float d = norm_p2_p1; float p3_p1[3]; vector_sub(P3,P1,p3_p1,3); float i = vector_dot(ex,p3_p1,3); float j = vector_dot(ey,p3_p1,3);
  float x = (r1*r1 - r2*r2 + d*d)/(2*d); float y = (r1*r1 - r3*r3 + i*i + j*j)/(2*j) - (i/j)*x; float z2 = r1*r1 - x*x - y*y; float z = (z2<0)?0.0f:sqrt(z2);
  float t1[3],t2[3],t3[3]; vector_scale(ex,x,t1,3); vector_scale(ey,y,t2,3); vector_scale(ez,z,t3,3); vector_add(P1,t1,out,3); vector_add(out,t2,out,3); vector_add(out,t3,out,3);
}

float angulo_direccion_xy(float* pos){ float x=pos[0]; float y=pos[1]; float ang_rad = atan2(x,y); return ang_rad*180.0/M_PI; }
float desenrollar_angulo(float previo,float actual){ float delta=actual-previo; if(delta>180) delta-=360; else if(delta<-180) delta+=360; return previo+delta; }
float limitar_cambio(float previo,float actual,float max_d){ float delta=actual-previo; if(delta>max_d) delta=max_d; else if(delta<-max_d) delta=-max_d; return previo+delta; }

bool debe_corregir(float angulo,float umbral){ return fabs(angulo) > umbral; }
float calcular_umbral_dinamico(float dist_mm,float um_min,float um_max,float dmin,float dmax){ if(dist_mm<=dmin) return um_min; else if(dist_mm>=dmax) return um_max; float f=(dist_mm-dmin)/(dmax-dmin); return um_min + f*(um_max-um_min); }

float suavizar_velocidad_avanzada(float v_act,float v_obj,float a_max,float d_max){ if(v_act < v_obj){ float delta=v_obj-v_act; float paso=a_max; if(paso>delta) paso=delta; return v_act+paso; } if(v_act>v_obj){ float delta=v_act-v_obj; float paso=d_max; if(paso>delta) paso=delta; return v_act-paso; } return v_act; }

static float clip(float v,float mn,float mx){ if(v<mn) return mn; if(v>mx) return mx; return v; }
VelocidadesDiferenciales calcular_velocidades_diferenciales(float v_lin,float ang_rel,float max_v){ VelocidadesDiferenciales r; r.giro_normalizado = clip(ang_rel/30.0f,-1.0f,1.0f); float vel_der_f=clip(v_lin*(1.0f - r.giro_normalizado),0,max_v); float vel_izq_f=clip(v_lin*(1.0f + r.giro_normalizado),0,max_v); r.vel_der=(int)vel_der_f; r.vel_izq=(int)vel_izq_f; return r; }
