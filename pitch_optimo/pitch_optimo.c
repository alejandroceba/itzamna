/*
 * pitch_optimo.c
 * Análisis de pitch óptimo para rotor en descenso (NACA 0012)
 * Traducido desde Pitch_optimo.ipynb
 *
 * Compilar:  gcc -O2 -Wall -o pitch_optimo pitch_optimo.c -lm
 * Ejecutar:  ./pitch_optimo
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* ─── Parámetros del sistema ─────────────────────────────────────────────── */
#define N_PALAS  3
#define M        0.420      /* Masa total [kg]                  */
#define R        0.22869    /* Radio exterior del rotor [m]     */
#define R0       0.04776    /* Radio inicio de pala [m]         */
#define C_CUERDA 0.0204     /* Cuerda de la pala [m]            */
#define RHO      1.225      /* Densidad del aire [kg/m³]        */
#define G        9.81       /* Gravedad [m/s²]                  */

/* ─── Discretización ────────────────────────────────────────────────────── */
/* alpha: -3 a 15 paso 0.5  →  37 valores                                   */
#define N_ALPHA  37
#define ALPHA_MIN  -3.0
#define ALPHA_STEP  0.5

/* Vd: 0.5 a 50, 200 puntos lineales                                         */
#define N_VD  200
#define VD_MIN  0.5
#define VD_MAX  50.0

/* ─── Constante M_PI si el compilador no la define ─────────────────────── */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── Prototipos ────────────────────────────────────────────────────────── */
static void guardar_csv_empuje  (double Vd[N_VD], double T[N_ALPHA][N_VD],
                                  double alpha[N_ALPHA]);
static void guardar_csv_rpm     (double Vd[N_VD], double Om[N_ALPHA][N_VD],
                                  double alpha[N_ALPHA]);
static void guardar_csv_efic    (double alpha[N_ALPHA], double efic[N_ALPHA]);
static void guardar_csv_vd_eq   (double alpha[N_ALPHA], double Vd_eq[N_ALPHA]);
static void guardar_csv_3d      (double alpha[N_ALPHA], double Vd[N_VD],
                                  double Om[N_ALPHA][N_VD]);

/* ═══════════════════════════════════════════════════════════════════════════
 *  main
 * ═════════════════════════════════════════════════════════════════════════*/
int main(void)
{
    /* ── Peso total ──────────────────────────────────────────────────────── */
    const double W = M * G;

    /* ── Tabla aerodinámica – Perfil NACA 0012 ───────────────────────────── */
    double alpha_deg[N_ALPHA], CL[N_ALPHA], CD[N_ALPHA], eficiencia[N_ALPHA];

    for (int i = 0; i < N_ALPHA; i++) {
        alpha_deg[i] = ALPHA_MIN + i * ALPHA_STEP;
        CL[i] = 2.0 * M_PI * (alpha_deg[i] * M_PI / 180.0); /* lineal */
        CD[i] = 0.008 + 0.01 * (CL[i] - 0.0) * (CL[i] - 0.0);
        eficiencia[i] = CL[i] / CD[i];
    }

    /* ── Vector de velocidades de descenso ───────────────────────────────── */
    double Vd_sensor[N_VD];
    for (int j = 0; j < N_VD; j++) {
        Vd_sensor[j] = VD_MIN + (VD_MAX - VD_MIN) * j / (N_VD - 1.0);
    }

    /* ── Rangos de radio ─────────────────────────────────────────────────── */
    const double R3r0 = R*R*R - R0*R0*R0;
    const double R4r0 = R*R*R*R - R0*R0*R0*R0;

    /* ── Matrices de resultados ──────────────────────────────────────────── */
    double T_matrix[N_ALPHA][N_VD];
    double Om[N_ALPHA][N_VD];

    for (int i = 0; i < N_ALPHA; i++) {
        for (int j = 0; j < N_VD; j++) {
            /* Ecuación A – velocidad angular */
            Om[i][j] = (4.0 * Vd_sensor[j] * CL[i] * R3r0)
                      / (3.0 * CD[i] * R4r0);
            /* Ecuación B – empuje */
            T_matrix[i][j] = ((double)N_PALAS / 6.0) * RHO * C_CUERDA
                             * CL[i] * Om[i][j]*Om[i][j] * R3r0;
        }
    }

    /* ── Velocidad de descenso de equilibrio ─────────────────────────────── */
    const double K_eq = (16.0 * N_PALAS / 54.0) * RHO * C_CUERDA
                        * (R3r0*R3r0*R3r0) / (R4r0*R4r0);

    double Vd_eq[N_ALPHA];

    printf("%-12s %-8s %-8s %-18s %s\n",
           "alpha [°]", "CL", "CD", "Vd_eq [m/s]", "Régimen");
    printf("%s\n", "-----------------------------------------------------------------");

    for (int i = 0; i < N_ALPHA; i++) {
        Vd_eq[i] = sqrt(W / (K_eq * (CL[i]*CL[i]*CL[i]) / (CD[i]*CD[i])));

        const char *regimen;
        if      (Vd_eq[i] > 8.0) regimen = "Caída rápida";
        else if (Vd_eq[i] > 3.0) regimen = "Descenso moderado";
        else                      regimen = "Descenso lento / estable";

        printf("%-12.0f %-8.3f %-8.5f %-18.3f %s\n",
               alpha_deg[i], CL[i], CD[i], Vd_eq[i], regimen);
    }

    /* ── Lógica de control ───────────────────────────────────────────────── */
    const double e    = 0.001;
    const double Vobj = 0.1;

    printf("\n--- Lógica de control ---\n");
    for (int i = 0; i < N_ALPHA; i++) {
        const char *accion = (Vd_eq[i] - Vobj > e)
                             ? "aumentar pitch"
                             : "disminuir pitch";
        printf("alpha = %-8.2f | Vd_eq = %-8.3f | acción = %s\n",
               alpha_deg[i], Vd_eq[i], accion);
    }

    /* ── Exportar datos para graficar ────────────────────────────────────── */
    guardar_csv_empuje (Vd_sensor, T_matrix,  alpha_deg);
    guardar_csv_rpm    (Vd_sensor, Om,         alpha_deg);
    guardar_csv_efic   (alpha_deg, eficiencia);
    guardar_csv_vd_eq  (alpha_deg, Vd_eq);
    guardar_csv_3d     (alpha_deg, Vd_sensor,  Om);

    printf("\nArchivos CSV generados:\n");
    printf("  empuje_vs_vd.csv     → Gráfica 1: Empuje vs Vel. descenso\n");
    printf("  rpm_vs_vd.csv        → Gráfica 2: RPM vs Vel. descenso\n");
    printf("  eficiencia.csv       → Gráfica 3: Eficiencia por ángulo\n");
    printf("  vd_equilibrio.csv    → Gráfica 4: Vd equilibrio vs Pitch\n");
    printf("  superficie_3d.csv    → Gráfica 5: Pitch / RPM / Vel. descenso\n");

    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Funciones auxiliares de exportación CSV
 * ═════════════════════════════════════════════════════════════════════════*/

/* Gráfica 1 ─ Empuje vs velocidad de descenso */
static void guardar_csv_empuje(double Vd[N_VD], double T[N_ALPHA][N_VD],
                                double alpha[N_ALPHA])
{
    FILE *f = fopen("empuje_vs_vd.csv", "w");
    if (!f) { perror("empuje_vs_vd.csv"); return; }

    fprintf(f, "Vd_m_s");
    for (int i = 0; i < N_ALPHA; i++) fprintf(f, ",alpha_%.1f", alpha[i]);
    fprintf(f, "\n");

    for (int j = 0; j < N_VD; j++) {
        fprintf(f, "%.4f", Vd[j]);
        for (int i = 0; i < N_ALPHA; i++) fprintf(f, ",%.6f", T[i][j]);
        fprintf(f, "\n");
    }
    fclose(f);
}

/* Gráfica 2 ─ RPM vs velocidad de descenso */
static void guardar_csv_rpm(double Vd[N_VD], double Om[N_ALPHA][N_VD],
                             double alpha[N_ALPHA])
{
    FILE *f = fopen("rpm_vs_vd.csv", "w");
    if (!f) { perror("rpm_vs_vd.csv"); return; }

    fprintf(f, "Vd_m_s");
    for (int i = 0; i < N_ALPHA; i++) fprintf(f, ",alpha_%.1f", alpha[i]);
    fprintf(f, "\n");

    for (int j = 0; j < N_VD; j++) {
        fprintf(f, "%.4f", Vd[j]);
        for (int i = 0; i < N_ALPHA; i++) {
            double rpm = Om[i][j] * 60.0 / (2.0 * M_PI);
            fprintf(f, ",%.4f", rpm);
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

/* Gráfica 3 ─ Eficiencia aerodinámica por ángulo */
static void guardar_csv_efic(double alpha[N_ALPHA], double efic[N_ALPHA])
{
    FILE *f = fopen("eficiencia.csv", "w");
    if (!f) { perror("eficiencia.csv"); return; }

    fprintf(f, "alpha_deg,CL_CD\n");
    for (int i = 0; i < N_ALPHA; i++)
        fprintf(f, "%.1f,%.6f\n", alpha[i], efic[i]);
    fclose(f);
}

/* Gráfica 4 ─ Velocidad de descenso de equilibrio vs pitch */
static void guardar_csv_vd_eq(double alpha[N_ALPHA], double Vd_eq[N_ALPHA])
{
    FILE *f = fopen("vd_equilibrio.csv", "w");
    if (!f) { perror("vd_equilibrio.csv"); return; }

    fprintf(f, "alpha_deg,Vd_eq_m_s\n");
    for (int i = 0; i < N_ALPHA; i++)
        fprintf(f, "%.1f,%.6f\n", alpha[i], Vd_eq[i]);
    fclose(f);
}

/* Gráfica 5 ─ Superficie 3D: pitch / vel. descenso / RPM */
static void guardar_csv_3d(double alpha[N_ALPHA], double Vd[N_VD],
                            double Om[N_ALPHA][N_VD])
{
    FILE *f = fopen("superficie_3d.csv", "w");
    if (!f) { perror("superficie_3d.csv"); return; }

    fprintf(f, "alpha_deg,Vd_m_s,omega_rad_s\n");
    for (int i = 0; i < N_ALPHA; i++)
        for (int j = 0; j < N_VD; j++)
            fprintf(f, "%.1f,%.4f,%.6f\n", alpha[i], Vd[j], Om[i][j]);
    fclose(f);
}
