/**
 * @file Filtros.h
 * @brief Conjunto de filtros y utilidades matemáticas migradas fielmente desde CarroClean (calculos.cpp / .h).
 *
 * Provee:
 *  - Filtro de media móvil multi‑sensor (hasta 3 sensores, ventana max 10)
 *  - Filtro de Kalman 1D aplicado de forma independiente a múltiples sensores
 *  - Limitador de variación entre muestras consecutivas
 *  - Operaciones vectoriales básicas y trilateración 3D simplificada
 *  - Utilidades de ángulo / dirección y umbral dinámico
 *  - Suavizado de velocidad con rampas de aceleración / desaceleración
 *  - Cálculo de velocidades diferenciales sin reversa
 *
 * Todos los algoritmos mantienen la lógica exacta del proyecto original para asegurar
 * reproducibilidad y comparabilidad de comportamiento.
 */
#pragma once
#include <Arduino.h>
#include <math.h>

namespace utils {

// ===================== MEDIA MOVIL =====================
/**
 * @brief Filtro de media móvil para múltiples sensores (hasta 3).
 *
 * Implementación compacta sin memoria dinámica. Cada sensor mantiene su propio
 * buffer circular. El tamaño máximo de ventana está acotado a 10 igual que el
 * código original.
 */
class MediaMovilMulti {
public:
    /**
     * @brief Inicializa la estructura interna.
     * @param n_sensores Número de sensores (máx 3)
     * @param ventana Tamaño de la ventana de media (capado a 10)
     */
    void init(int n_sensores, int ventana) {
        n_ = n_sensores; win_ = ventana;
        if (win_ > 10) win_ = 10; // límite igual al código original
        for (int i=0;i<n_ && i<3;i++) {
            idx_[i]=0; count_[i]=0;
            for (int j=0;j<win_;j++) buf_[i][j]=0.0f;
        }
    }
    /**
     * @brief Actualiza el filtro con nuevas lecturas y devuelve la media filtrada.
     * @param nuevos Array de entradas (n_sensores valores)
     * @param out Array de salida (n_sensores valores filtrados)
     */
    void actualizar(float* nuevos, float* out) {
        for (int i=0;i<n_ && i<3;i++) {
            buf_[i][idx_[i]] = nuevos[i];
            idx_[i] = (idx_[i]+1) % win_;
            if (count_[i] < win_) count_[i]++;
            float suma=0.0f; for(int j=0;j<count_[i];j++) suma+=buf_[i][j];
            out[i] = suma / count_[i];
        }
    }
private:
    int n_=3; int win_=3;
    float buf_[3][10];
    int idx_[3]; int count_[3];
};

// ===================== KALMAN SIMPLE =====================
/**
 * @brief Filtro de Kalman 1D aplicado de forma independiente a múltiples canales.
 *
 * Modelo muy simple: estado = valor; sólo ajusta la incertidumbre con Q y R.
 */
class KalmanMulti {
public:
    /**
     * @brief Inicializa estados y covarianzas.
     * @param n Número de canales (máx 3)
     * @param Q Varianza de proceso
     * @param R Varianza de medición
     */
    void init(int n, float Q, float R) {
        n_ = n; Q_=Q; R_=R;
        for(int i=0;i<n_ && i<3;i++){ x_[i]=0.0f; P_[i]=1.0f; }
    }
    /**
     * @brief Aplica un paso de predicción/actualización para cada canal.
     * @param z Medidas de entrada
     * @param out Estados filtrados resultantes
     */
    void filtrar(float* z, float* out) {
        for(int i=0;i<n_ && i<3;i++) {
            P_[i] += Q_;
            float K = P_[i] / (P_[i] + R_);
            x_[i] = x_[i] + K*(z[i]-x_[i]);
            P_[i] = (1 - K)*P_[i];
            out[i] = x_[i];
        }
    }
    /**
     * @brief Fuerza el estado a valores conocidos y reinicia P.
     * @param z Nuevos estados
     * @param P_init Covarianza inicial
     */
    void forceState(float* z, float P_init){ for(int i=0;i<n_ && i<3;i++){ x_[i]=z[i]; P_[i]=P_init; } }
    void setQ(float Q){ Q_=Q; }
    float getQ() const { return Q_; }
private:
    int n_=3; float Q_=0.01f; float R_=1.0f; float x_[3]; float P_[3];
};

// ===================== LIMITAR VARIACION =====================
/**
 * @brief Limita la variación entre dos vectores de distancias.
 * @param dist_actual Lecturas actuales
 * @param dist_prev Lecturas previas
 * @param out Resultado limitado
 * @param n Número de elementos
 * @param max_delta Cambio máximo permitido por elemento
 */
inline void limitar_variacion(float* dist_actual, float* dist_prev, float* out, int n, float max_delta) {
    for(int i=0;i<n;i++) {
        float delta = dist_actual[i]-dist_prev[i];
        if (delta>max_delta) delta=max_delta; else if (delta<-max_delta) delta=-max_delta;
        out[i] = dist_prev[i]+delta;
    }
}

// ===================== VECTORES / TRILATERACION =====================
inline float v_norm(const float* v,int sz){ float s=0; for(int i=0;i<sz;i++) s+=v[i]*v[i]; return sqrtf(s);} 
inline void v_sub(const float* a,const float* b,float* r,int sz){ for(int i=0;i<sz;i++) r[i]=a[i]-b[i]; }
inline void v_add(const float* a,const float* b,float* r,int sz){ for(int i=0;i<sz;i++) r[i]=a[i]+b[i]; }
inline void v_scale(const float* v,float k,float* r,int sz){ for(int i=0;i<sz;i++) r[i]=v[i]*k; }
inline float v_dot(const float* a,const float* b,int sz){ float s=0; for(int i=0;i<sz;i++) s+=a[i]*b[i]; return s; }
inline void v_cross3(const float* a,const float* b,float* r){ r[0]=a[1]*b[2]-a[2]*b[1]; r[1]=a[2]*b[0]-a[0]*b[2]; r[2]=a[0]*b[1]-a[1]*b[0]; }

/**
 * @brief Trilateración 3D simplificada para 3 sensores.
 * @param sensor_pos Posiciones [3][3] de los sensores
 * @param dist Distancias medidas (3)
 * @param out Posición estimada (x,y,z)
 * @note Implementación simplificada replicando la lógica original (no maneja degeneraciones).
 */
inline void trilateracion_3d(const float sensor_pos[3][3], float* dist, float* out) {
    float P1[3]={sensor_pos[0][0],sensor_pos[0][1],sensor_pos[0][2]};
    float P2[3]={sensor_pos[1][0],sensor_pos[1][1],sensor_pos[1][2]};
    float P3[3]={sensor_pos[2][0],sensor_pos[2][1],sensor_pos[2][2]};
    float r1=dist[0], r2=dist[1], r3=dist[2];
    float p2_p1[3]; v_sub(P2,P1,p2_p1,3); float d=v_norm(p2_p1,3); float ex[3]; v_scale(p2_p1,1.0f/d,ex,3);
    float ey_dir[3]={0,1,0}; float ey[3]; v_scale(ey_dir,1.0f/v_norm(ey_dir,3),ey,3);
    float ez[3]; v_cross3(ex,ey,ez);
    float p3_p1[3]; v_sub(P3,P1,p3_p1,3); float i=v_dot(ex,p3_p1,3); float j=v_dot(ey,p3_p1,3);
    float x=(r1*r1 - r2*r2 + d*d)/(2*d);
    float y=(r1*r1 - r3*r3 + i*i + j*j)/(2*j) - (i/j)*x;
    float z2 = r1*r1 - x*x - y*y; float z = z2<0?0:sqrtf(z2);
    float t1[3],t2[3],t3[3]; v_scale(ex,x,t1,3); v_scale(ey,y,t2,3); v_scale(ez,z,t3,3);
    v_add(P1,t1,out,3); v_add(out,t2,out,3); v_add(out,t3,out,3);
}

// ===================== ANGULOS =====================
/**
 * @brief Calcula el ángulo (grados) de dirección usando eje Y como referencia.
 * @param pos_tag Vector posición tag [x,y,(z opcional)]
 * @return Ángulo en grados (-180,180]
 */
inline float angulo_direccion_xy(float* pos_tag) {
    float x=pos_tag[0]; float y=pos_tag[1];
    float ang = atan2f(x,y) * 180.0f / PI; // mismo signo que original
    return ang;
}
/**
 * @brief Desenrolla un ángulo evitando saltos mayores a 180°.
 */
inline float desenrollar_angulo(float previo, float actual) {
    float delta = actual-previo; if (delta>180) delta-=360; else if (delta<-180) delta+=360; return previo+delta; }
/** @brief Limita el cambio angular máximo permitido. */
inline float limitar_cambio(float previo, float actual, float max_delta){ float d=actual-previo; if(d>max_delta)d=max_delta; else if(d<-max_delta)d=-max_delta; return previo+d; }
/** @brief Indica si el ángulo excede un umbral absoluto. */
inline bool debe_corregir(float angulo,float umbral){ return fabsf(angulo)>umbral; }
/** @brief Umbral dinámico lineal entre distancias (interpolación). */
inline float calcular_umbral_dinamico(float dist,float umin,float umax,float dmin,float dmax){ if(dist<=dmin)return umin; if(dist>=dmax)return umax; float f=(dist-dmin)/(dmax-dmin); return umin+f*(umax-umin);} 

// ===================== SUAVIZADO VELOCIDAD =====================
/**
 * @brief Aplica rampa de aceleración/desaceleración hacia velocidad objetivo.
 */
inline float suavizar_velocidad_avanzada(float v_act, float v_obj, float acel, float desacel){
    if (v_act < v_obj){ float delta=v_obj-v_act; float paso=acel; if(paso>delta) paso=delta; return v_act+paso; }
    if (v_act > v_obj){ float delta=v_act-v_obj; float paso=desacel; if(paso>delta) paso=delta; return v_act-paso; }
    return v_act;
}

// ===================== DIFERENCIAL =====================
inline float clip(float v,float mn,float mx){ if(v<mn) return mn; if(v>mx) return mx; return v; }
/**
 * @brief Resultado del cálculo diferencial (sin reversa).
 */
struct VelocidadesDiferenciales { int vel_izq; int vel_der; float giro_normalizado; };
/**
 * @brief Calcula velocidades diferenciales limitadas (no se permite velocidad negativa).
 * @param v_lineal Velocidad lineal base
 * @param angulo_relativo Ángulo relativo (grados) a corregir
 * @param max_v Límite superior PWM/velocidad
 */
inline VelocidadesDiferenciales calcular_velocidades_diferenciales(float v_lineal,float angulo_relativo,float max_v){
    VelocidadesDiferenciales r; r.giro_normalizado=clip(angulo_relativo/30.0f,-1.0f,1.0f);
    float vel_der_f=clip(v_lineal*(1.0f-r.giro_normalizado),0,max_v);
    float vel_izq_f=clip(v_lineal*(1.0f+r.giro_normalizado),0,max_v);
    r.vel_der=(int)vel_der_f; r.vel_izq=(int)vel_izq_f; return r;
}

} // namespace utils
