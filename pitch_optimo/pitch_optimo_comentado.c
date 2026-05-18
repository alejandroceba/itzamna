/*
 * pitch_optimo_comentado.c
 *
 * Version documentada de pitch_optimo.c.
 *
 * Objetivo:
 *   Reproducir en C el flujo de la libreta Pitch_optimo.ipynb:
 *
 *   1. Definir parametros fisicos del rotor.
 *   2. Crear la tabla aerodinamica del perfil NACA 0012.
 *   3. Barrer velocidades de descenso.
 *   4. Calcular velocidad angular y empuje para cada pitch.
 *   5. Calcular velocidad de equilibrio.
 *   6. Mostrar una posible logica de control.
 *   7. Exportar CSV para graficar los mismos resultados que en Python.
 *
 * Compilar en PowerShell con MSYS2 UCRT64:
 *
 *   $env:Path = "C:\msys64\ucrt64\bin;$env:Path"
 *   gcc -O2 -Wall pitch_optimo_comentado.c -o pitch_optimo_comentado.exe -lm
 *   .\pitch_optimo_comentado.exe
 *
 * Nota:
 *   Esta version conserva la misma logica numerica que pitch_optimo.c.
 *   Los comentarios explican el flujo paso a paso.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* ============================================================================
 * CELDA 1 EQUIVALENTE: Parametros de entrada del sistema
 * ============================================================================
 *
 * Estos valores corresponden a la celda de la libreta donde se definian:
 *
 *   N_palas, m, R, r0, c, rho, g
 *
 * En C se usan macros porque son constantes conocidas en tiempo de compilacion.
 */

#define N_PALAS  3          /* Numero de palas del rotor. */
#define M        0.420      /* Masa total del sistema [kg]. */
#define R        0.22869    /* Radio exterior del rotor [m]. */
#define R0       0.04776    /* Radio donde inicia la pala [m]. */
#define C_CUERDA 0.0204     /* Cuerda de la pala [m]. */
#define RHO      1.225      /* Densidad del aire [kg/m^3]. */
#define G        9.81       /* Gravedad [m/s^2]. */

/* ============================================================================
 * CELDAS 2 Y 3 EQUIVALENTES: Discretizacion numerica
 * ============================================================================
 *
 * En Python:
 *
 *   alpha = np.arange(-3, 15.5, 0.5)
 *   Vd_sensor = np.linspace(0.5, 50, 200)
 *
 * En C no existen directamente np.arange ni np.linspace, asi que se definen
 * los limites y el numero de puntos. Luego se llenan los arreglos con ciclos.
 */

#define N_ALPHA     37
#define ALPHA_MIN  -3.0
#define ALPHA_STEP  0.5

#define N_VD   200
#define VD_MIN 0.5
#define VD_MAX 50.0

/*
 * Algunos compiladores en Windows no definen M_PI por defecto.
 * Esta constante se necesita para convertir grados a radianes.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ============================================================================
 * Prototipos de funciones auxiliares
 * ============================================================================
 *
 * Estas funciones no cambian la fisica del modelo.
 * Solo escriben los resultados en archivos CSV para graficarlos despues.
 */

static void guardar_csv_empuje(double Vd[N_VD],
                               double T[N_ALPHA][N_VD],
                               double alpha[N_ALPHA]);

static void guardar_csv_rpm(double Vd[N_VD],
                            double Om[N_ALPHA][N_VD],
                            double alpha[N_ALPHA]);

static void guardar_csv_efic(double alpha[N_ALPHA],
                             double efic[N_ALPHA]);

static void guardar_csv_vd_eq(double alpha[N_ALPHA],
                              double Vd_eq[N_ALPHA]);

static void guardar_csv_3d(double alpha[N_ALPHA],
                           double Vd[N_VD],
                           double Om[N_ALPHA][N_VD]);

/* ============================================================================
 * Funcion principal
 * ============================================================================
 *
 * El programa empieza aqui.
 * El orden interno sigue la estructura de la libreta original.
 */

int main(void)
{
    /* ------------------------------------------------------------------------
     * CELDA: Peso total
     * ------------------------------------------------------------------------
     *
     * En Python:
     *
     *   W = m*g
     *
     * W es el peso total del sistema. Se usa despues para encontrar la
     * velocidad de descenso de equilibrio, donde el empuje iguala al peso.
     */

    const double W = M * G;

    /* ------------------------------------------------------------------------
     * CELDA: Tabla aerodinamica - Perfil NACA 0012
     * ------------------------------------------------------------------------
     *
     * En Python:
     *
     *   alpha = np.arange(-3, 15.5, 0.5)
     *   Cl = 2*np.pi*np.radians(alpha)
     *   Cd = 0.008 + 0.01*(Cl - 0)**2
     *   eficiencia = CL/CD
     *
     * Aqui se calcula lo mismo, pero elemento por elemento.
     *
     * alpha_deg:
     *   Angulo de pitch en grados.
     *
     * CL:
     *   Coeficiente de sustentacion lineal.
     *
     * CD:
     *   Coeficiente de arrastre usando una polar parabolica simplificada.
     *
     * eficiencia:
     *   Relacion CL/CD.
     */

    double alpha_deg[N_ALPHA];
    double CL[N_ALPHA];
    double CD[N_ALPHA];
    double eficiencia[N_ALPHA];

    for (int i = 0; i < N_ALPHA; i++) {
        /* Equivalente a alpha[i] = -3 + i*0.5. */
        alpha_deg[i] = ALPHA_MIN + i * ALPHA_STEP;

        /* Conversion de grados a radianes: rad = deg*pi/180. */
        CL[i] = 2.0 * M_PI * (alpha_deg[i] * M_PI / 180.0);

        /* Polar parabolica usada en la libreta. */
        CD[i] = 0.008 + 0.01 * (CL[i] - 0.0) * (CL[i] - 0.0);

        /* Eficiencia aerodinamica para cada angulo. */
        eficiencia[i] = CL[i] / CD[i];
    }

    /* ------------------------------------------------------------------------
     * CELDA: Vector de velocidades de descenso
     * ------------------------------------------------------------------------
     *
     * En Python:
     *
     *   Vd_sensor = np.linspace(0.5, 50, 200)
     *
     * np.linspace incluye el primer y ultimo valor.
     * Por eso el paso se calcula dividiendo entre N_VD - 1.
     */

    double Vd_sensor[N_VD];

    for (int j = 0; j < N_VD; j++) {
        Vd_sensor[j] = VD_MIN + (VD_MAX - VD_MIN) * j / (N_VD - 1.0);
    }

    /* ------------------------------------------------------------------------
     * CELDA: Terminos radiales
     * ------------------------------------------------------------------------
     *
     * En Python:
     *
     *   R3r0 = R**3 - r0**3
     *   R4r0 = R**4 - r0**4
     *
     * Estos terminos aparecen en las ecuaciones de velocidad angular y empuje.
     */

    const double R3r0 = R*R*R - R0*R0*R0;
    const double R4r0 = R*R*R*R - R0*R0*R0*R0;

    /* ------------------------------------------------------------------------
     * CELDA: Matrices de salida
     * ------------------------------------------------------------------------
     *
     * En Python:
     *
     *   T_matrix = np.zeros((n_alpha, n_Vd))
     *   Om = np.zeros((n_alpha, n_Vd))
     *
     * Cada fila corresponde a un angulo alpha.
     * Cada columna corresponde a una velocidad de descenso Vd.
     *
     * T_matrix[i][j]:
     *   Empuje para alpha i y velocidad Vd j.
     *
     * Om[i][j]:
     *   Velocidad angular en rad/s para alpha i y velocidad Vd j.
     */

    double T_matrix[N_ALPHA][N_VD];
    double Om[N_ALPHA][N_VD];

    /* ------------------------------------------------------------------------
     * CELDA: Barrido de alpha y Vd
     * ------------------------------------------------------------------------
     *
     * En Python se usaban dos ciclos anidados:
     *
     *   for i in range(n_alpha):
     *       for j in range(n_Vd):
     *           Om[i, j] = ...
     *           T_matrix[i, j] = ...
     *
     * Aqui se usa exactamente la misma estructura.
     */

    for (int i = 0; i < N_ALPHA; i++) {
        for (int j = 0; j < N_VD; j++) {
            /*
             * Ecuacion A: velocidad angular.
             *
             * Python:
             *
             *   Om[i, j] = (4 * Vd_sensor[j] * CL[i] * R3r0)
             *              / (3 * CD[i] * R4r0)
             */
            Om[i][j] = (4.0 * Vd_sensor[j] * CL[i] * R3r0)
                      / (3.0 * CD[i] * R4r0);

            /*
             * Ecuacion B: empuje.
             *
             * Python:
             *
             *   T_matrix[i, j] = (N_palas / 6) * rho * c
             *                    * CL[i] * Om[i, j]**2 * R3r0
             *
             * En C, elevar al cuadrado se escribe como Om*Om.
             */
            T_matrix[i][j] = ((double)N_PALAS / 6.0) * RHO * C_CUERDA
                             * CL[i] * Om[i][j] * Om[i][j] * R3r0;
        }
    }

    /* ------------------------------------------------------------------------
     * CELDA: Equilibrio para la velocidad de descenso
     * ------------------------------------------------------------------------
     *
     * En Python:
     *
     *   K_eq = (16*N_palas/54)*rho*c*(R3r0**3/R4r0**2)
     *   Vd_eq[i] = sqrt(W / (K_eq*(CL[i]**3/CD[i]**2)))
     *
     * Interpretacion:
     *   Vd_eq es la velocidad de descenso donde el empuje del rotor iguala
     *   al peso del sistema. Es decir, el sistema queda en equilibrio vertical.
     *
     * Casos especiales:
     *   Para CL negativo, la raiz puede no ser real y aparece NaN.
     *   Para CL = 0, aparece infinito por division entre cero.
     *   Esto coincide con el comportamiento de la libreta original.
     */

    const double K_eq = (16.0 * N_PALAS / 54.0) * RHO * C_CUERDA
                        * (R3r0 * R3r0 * R3r0) / (R4r0 * R4r0);

    double Vd_eq[N_ALPHA];

    printf("%-12s %-8s %-8s %-18s %s\n",
           "alpha [deg]", "CL", "CD", "Vd_eq [m/s]", "Regimen");
    printf("%s\n", "-----------------------------------------------------------------");

    for (int i = 0; i < N_ALPHA; i++) {
        const double clCubedOverCdSquared = (CL[i] * CL[i] * CL[i])
                                            / (CD[i] * CD[i]);

        Vd_eq[i] = sqrt(W / (K_eq * clCubedOverCdSquared));

        /*
         * Clasificacion cualitativa usada en la libreta.
         * No altera los calculos; solo ayuda a interpretar la tabla.
         */
        const char *regimen;

        if (Vd_eq[i] > 8.0) {
            regimen = "Caida rapida";
        } else if (Vd_eq[i] > 3.0) {
            regimen = "Descenso moderado";
        } else {
            regimen = "Descenso lento / estable";
        }

        printf("%-12.0f %-8.3f %-8.5f %-18.3f %s\n",
               alpha_deg[i], CL[i], CD[i], Vd_eq[i], regimen);
    }

    /* ------------------------------------------------------------------------
     * CELDA: Posible logica de control
     * ------------------------------------------------------------------------
     *
     * En Python:
     *
     *   e = 0.001
     *   Vobj = 0.1
     *   wr = 'aumentar pitch' if Vd_eq[i] - Vobj > e else 'disminuir pitch'
     *
     * Esta seccion compara la velocidad de equilibrio contra una velocidad
     * objetivo. Si la diferencia supera la tolerancia, se recomienda aumentar
     * pitch; en caso contrario, disminuir pitch.
     */

    const double e = 0.001;
    const double Vobj = 0.1;

    printf("\n--- Logica de control ---\n");

    for (int i = 0; i < N_ALPHA; i++) {
        const char *accion = (Vd_eq[i] - Vobj > e)
                             ? "aumentar pitch"
                             : "disminuir pitch";

        printf("alpha = %-8.2f | Vd_eq = %-8.3f | accion = %s\n",
               alpha_deg[i], Vd_eq[i], accion);
    }

    /* ------------------------------------------------------------------------
     * CELDA: Exportar datos para graficar
     * ------------------------------------------------------------------------
     *
     * La libreta original graficaba directamente los arreglos en memoria.
     * En C es mas practico guardar esos arreglos como CSV y luego graficarlos
     * desde Python/Jupyter.
     *
     * Archivos generados:
     *
     *   empuje_vs_vd.csv
     *   rpm_vs_vd.csv
     *   eficiencia.csv
     *   vd_equilibrio.csv
     *   superficie_3d.csv
     */

    guardar_csv_empuje(Vd_sensor, T_matrix, alpha_deg);
    guardar_csv_rpm(Vd_sensor, Om, alpha_deg);
    guardar_csv_efic(alpha_deg, eficiencia);
    guardar_csv_vd_eq(alpha_deg, Vd_eq);
    guardar_csv_3d(alpha_deg, Vd_sensor, Om);

    printf("\nArchivos CSV generados:\n");
    printf("  empuje_vs_vd.csv     -> Grafica 1: Empuje vs Vel. descenso\n");
    printf("  rpm_vs_vd.csv        -> Grafica 2: RPM vs Vel. descenso\n");
    printf("  eficiencia.csv       -> Grafica 3: Eficiencia por angulo\n");
    printf("  vd_equilibrio.csv    -> Grafica 4: Vd equilibrio vs Pitch\n");
    printf("  superficie_3d.csv    -> Grafica 5: Pitch / RPM / Vel. descenso\n");

    return 0;
}

/* ============================================================================
 * Funciones auxiliares de exportacion CSV
 * ============================================================================
 *
 * Todas las funciones siguen una idea simple:
 *
 *   1. Abrir un archivo en modo escritura.
 *   2. Escribir encabezados.
 *   3. Recorrer arreglos y escribir valores separados por comas.
 *   4. Cerrar el archivo.
 *
 * Si fopen falla, perror muestra el nombre del archivo que no pudo abrirse.
 */

/* ---------------------------------------------------------------------------
 * CSV 1: Empuje vs velocidad de descenso
 * ---------------------------------------------------------------------------
 *
 * Formato:
 *
 *   Vd_m_s,alpha_-3.0,alpha_-2.5,...
 *   0.5000,...valores de empuje...
 *
 * Cada columna despues de Vd_m_s es una curva de alpha.
 */

static void guardar_csv_empuje(double Vd[N_VD],
                               double T[N_ALPHA][N_VD],
                               double alpha[N_ALPHA])
{
    FILE *f = fopen("empuje_vs_vd.csv", "w");

    if (!f) {
        perror("empuje_vs_vd.csv");
        return;
    }

    fprintf(f, "Vd_m_s");
    for (int i = 0; i < N_ALPHA; i++) {
        fprintf(f, ",alpha_%.1f", alpha[i]);
    }
    fprintf(f, "\n");

    for (int j = 0; j < N_VD; j++) {
        fprintf(f, "%.4f", Vd[j]);

        for (int i = 0; i < N_ALPHA; i++) {
            fprintf(f, ",%.6f", T[i][j]);
        }

        fprintf(f, "\n");
    }

    fclose(f);
}

/* ---------------------------------------------------------------------------
 * CSV 2: RPM vs velocidad de descenso
 * ---------------------------------------------------------------------------
 *
 * Om esta en rad/s, porque asi se calculaba en la libreta.
 * Para graficar RPM se convierte con:
 *
 *   RPM = Om * 60 / (2*pi)
 */

static void guardar_csv_rpm(double Vd[N_VD],
                            double Om[N_ALPHA][N_VD],
                            double alpha[N_ALPHA])
{
    FILE *f = fopen("rpm_vs_vd.csv", "w");

    if (!f) {
        perror("rpm_vs_vd.csv");
        return;
    }

    fprintf(f, "Vd_m_s");
    for (int i = 0; i < N_ALPHA; i++) {
        fprintf(f, ",alpha_%.1f", alpha[i]);
    }
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

/* ---------------------------------------------------------------------------
 * CSV 3: Eficiencia aerodinamica por angulo
 * ---------------------------------------------------------------------------
 *
 * Guarda la relacion CL/CD para cada alpha.
 */

static void guardar_csv_efic(double alpha[N_ALPHA],
                             double efic[N_ALPHA])
{
    FILE *f = fopen("eficiencia.csv", "w");

    if (!f) {
        perror("eficiencia.csv");
        return;
    }

    fprintf(f, "alpha_deg,CL_CD\n");

    for (int i = 0; i < N_ALPHA; i++) {
        fprintf(f, "%.1f,%.6f\n", alpha[i], efic[i]);
    }

    fclose(f);
}

/* ---------------------------------------------------------------------------
 * CSV 4: Velocidad de descenso de equilibrio vs pitch
 * ---------------------------------------------------------------------------
 *
 * Guarda Vd_eq para cada alpha.
 * Puede incluir NaN o inf, igual que la libreta original.
 */

static void guardar_csv_vd_eq(double alpha[N_ALPHA],
                              double Vd_eq[N_ALPHA])
{
    FILE *f = fopen("vd_equilibrio.csv", "w");

    if (!f) {
        perror("vd_equilibrio.csv");
        return;
    }

    fprintf(f, "alpha_deg,Vd_eq_m_s\n");

    for (int i = 0; i < N_ALPHA; i++) {
        fprintf(f, "%.1f,%.6f\n", alpha[i], Vd_eq[i]);
    }

    fclose(f);
}

/* ---------------------------------------------------------------------------
 * CSV 5: Superficie 3D
 * ---------------------------------------------------------------------------
 *
 * Guarda una tabla larga con:
 *
 *   alpha_deg, Vd_m_s, omega_rad_s
 *
 * Esto permite reconstruir una superficie 3D en Jupyter:
 *
 *   eje X: alpha
 *   eje Y: velocidad de descenso
 *   eje Z: omega
 */

static void guardar_csv_3d(double alpha[N_ALPHA],
                           double Vd[N_VD],
                           double Om[N_ALPHA][N_VD])
{
    FILE *f = fopen("superficie_3d.csv", "w");

    if (!f) {
        perror("superficie_3d.csv");
        return;
    }

    fprintf(f, "alpha_deg,Vd_m_s,omega_rad_s\n");

    for (int i = 0; i < N_ALPHA; i++) {
        for (int j = 0; j < N_VD; j++) {
            fprintf(f, "%.1f,%.4f,%.6f\n", alpha[i], Vd[j], Om[i][j]);
        }
    }

    fclose(f);
}
