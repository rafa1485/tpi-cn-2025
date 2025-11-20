// =============================================================================
// UNIVERSIDAD ADVENTISTA DEL PLATA
// MATERIA: Cálculo numérico
// AUTORES: Adriel San Martín y Fabricio Pereira
//
// DESCRIPCIÓN: 
// Simulación y optimización del costo energético de climatización (calefacción
// y refrigeración) utilizando el método de Euler para la ecuación diferencial
// de temperatura y Descenso de Gradiente para la optimización de horarios.
// =============================================================================

clear
clc

// =============================================================================
// 1. PARÁMETROS DEL SISTEMA
// =============================================================================

// --- Parámetros del perfil externo (Condiciones ambientales) ---
TAmbMax      = 24;   // Máxima Temperatura Ambiente [°C]
TAmbMin      = 10;   // Mínima Temperatura Ambiente [°C]
InicioSubida = 6;    // Hora inicio incremento temperatura [h]
FinSubida    = 11;   // Hora fin incremento temperatura [h]
InicioBajada = 14;   // Hora inicio descenso temperatura [h]
FinBajada    = 19;   // Hora fin descenso temperatura [h]

// --- Parámetros de la Edificación ---
superficieEdificacion = 100; // [m2]
superficiePiso        = 70;  // [m2]
espesorEdificacion    = 0.3; // [m]

// Propiedades térmicas de la estructura
coeficienteConductanciaEdificacion = 0.4 / espesorEdificacion; // [W/K/m2]
conductanciaEdificacion            = coeficienteConductanciaEdificacion * superficieEdificacion; // [W/K]

// Propiedades térmicas del piso
espesorAislacionPiso        = 0.05; // [m]
coeficienteConductanciaPiso = 0.02 / espesorAislacionPiso; // [W/K/m2]
conductanciaPiso            = superficiePiso * coeficienteConductanciaPiso; // [W/K]

// Masa térmica y capacidad calorífica
masaUnitaria                  = 150; // [kg/m2]
capacidadCalorificaEspecifica = 800; // [J/kg/K]
capacidadCalorificaUnitaria   = masaUnitaria * capacidadCalorificaEspecifica; // [J/K/m2]
capacidadCalorificaEdificio   = capacidadCalorificaUnitaria * superficiePiso; // [J/K]

// Convección
h = 18; // Coeficiente convección (viento 3 m/s) [W/m2K]
conductanciaConveccionEdificacion = h * superficieEdificacion;

// --- Parámetros del Sistema de Calefacción ---
potenciaCalefaccionUnitaria = 10; // [W/m2]
potenciaCalefaccion         = potenciaCalefaccionUnitaria * superficiePiso; // [W]
precioEnergiaCalefaccion    = 0.045 / 1000; // [USD/Wh] (Electricidad)

// (Cálculo alternativo con GAS - Comentado para futura referencia)
// poderCalorificoGas = 10.8; // [kWh/m3]
// precioM3Gas = 180; // [$/m3]
// precio_energia_Gas_Pesos_kWh = precioM3Gas / poderCalorificoGas / 0.8; 
// precioDolar_Pesos = 1500;
// precio_energia_Gas_USD_kWh = precio_energia_Gas_Pesos_kWh / precioDolar_Pesos;
// precio_energia_Gas_USD_Wh = precio_energia_Gas_USD_kWh / 1000;

// --- Parámetros del Sistema de Refrigeración ---
potenciaRefrigeracionUnitaria = 5; // [W/m2]
potenciaRefrigeracion         = potenciaRefrigeracionUnitaria * superficiePiso; // [W]
precioEnergiaRefrigeracion    = 0.12 / 1000; // [USD/Wh]


// =============================================================================
// 2. FUNCIONES DEL MODELO FÍSICO (Termodinámica)
// =============================================================================

// -----------------------------------------------------------------------------
// Función: T_exterior
// Descripción: Modela la temperatura ambiente en función de la hora del día
// -----------------------------------------------------------------------------
function T_ext = T_exterior(t)
    t_h = t / 3600; // Tiempo en horas
    if t <= InicioSubida * 3600 then
        T_ext = TAmbMin;
    elseif t <= FinSubida * 3600 then
        // Subida lineal
        T_ext = ((TAmbMax - TAmbMin) / (FinSubida - InicioSubida)) * (t_h - InicioSubida) + TAmbMin;
    elseif t <= InicioBajada * 3600 then
        T_ext = TAmbMax;
    elseif t <= FinBajada * 3600 then
        // Bajada lineal
        T_ext = ((TAmbMin - TAmbMax) / (FinBajada - InicioBajada)) * (t_h - InicioBajada) + TAmbMax;
    else
        T_ext = TAmbMin;
    end
endfunction

// -----------------------------------------------------------------------------
// Funciones de Flujo de Calor (Q)
// -----------------------------------------------------------------------------

// Flujo desde el suelo (constante 15°C)
function Qp = Q_piso(T_int)
    Qp = conductanciaPiso * (15 - T_int);
endfunction

// Flujo a través de paredes/techo (Conducción + Convección)
function Qe = Q_edif(t, T_int)
    T_ext = T_exterior(t);
    Re = 1 / conductanciaEdificacion;            // Resistencia conducción
    Rc = 1 / conductanciaConveccionEdificacion;  // Resistencia convección
    conductanciaTotalEdificacion = 1 / (Re + Rc);
    Qe = conductanciaTotalEdificacion * (T_ext - T_int);
endfunction

// Potencia entregada por Calefacción
function Qc = Q_calef(t, hr_ini_cal, hr_cal)
    hr_fin_cal = hr_ini_cal + hr_cal;
    if (t/3600) >= hr_ini_cal && (t/3600) <= hr_fin_cal then
        Qc = potenciaCalefaccion;
    else
        Qc = 0;
    end
endfunction

// Potencia entregada por Refrigeración
function Qr = Q_refri(t, hr_ini_ref, hr_ref)
    hr_fin_ref = hr_ini_ref + hr_ref;
    if (t/3600) >= hr_ini_ref && (t/3600) <= hr_fin_ref then
        Qr = -potenciaRefrigeracion;
    else
        Qr = 0;
    end
endfunction

// Sumatoria total de flujos térmicos
function Qt = Q_total(t, T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    Qp = Q_piso(T_int);
    Qe = Q_edif(t, T_int);
    Qc = Q_calef(t, hr_ini_cal, hr_cal);
    Qr = Q_refri(t, hr_ini_ref, hr_ref);
    Qt = Qp + Qe + Qc + Qr;
endfunction

// -----------------------------------------------------------------------------
// Función Diferencial (dT/dt) para Método de Euler
// -----------------------------------------------------------------------------
function dT = f(t, T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    dT = Q_total(t, T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref) / capacidadCalorificaEdificio;
endfunction

// -----------------------------------------------------------------------------
// Función de Integración Numérica (Regla del Trapecio)
// -----------------------------------------------------------------------------
function I = funcion_integral(t, Q)
    if length(t) < 2 then
        I = 0;
        return
    end
    dt = (t(2:$) - t(1:$-1))'; // Vector fila de diferenciales de tiempo
    mid = (Q(1:$-1) + Q(2:$)) .* dt / 2;
    I = sum(mid);
endfunction


// =============================================================================
// 3. FUNCIONES DE VISUALIZACIÓN Y CÁLCULO DE COSTOS
// =============================================================================

// -----------------------------------------------------------------------------
// Función: grafico_salida
// Descripción: Genera la ventana gráfica con Temperaturas, Potencias y Costos
// -----------------------------------------------------------------------------
function grafico_salida(t, T, Qc, Qr, costoRefrigeracion, costoCalefaccion, costoTotal)
    figure();
    
    // Subplot 1: Temperatura
    subplot(4,1,1);
    plot(t/3600, T);
    xlabel('Hora (h)');
    ylabel('T interna (°C)');
    title('Evolución de la temperatura interior');
    
    // Subplot 2: Calefacción
    subplot(4,1,2);
    plot(t/3600, Qc, 'r');
    xlabel('Hora (h)');
    ylabel('Qc (W)');
    title('Potencia de calefacción (Qc)');
    
    // Subplot 3: Refrigeración
    subplot(4,1,3);
    plot(t/3600, Qr, 'b');
    xlabel('Hora (h)');
    ylabel('Qr (W)');
    title('Potencia de refrigeración (Qr)');
    
    // Subplot 4: Resumen de Costos
    subplot(4,1,4);
    xstring(0.1, 0.75, "Costo Calefacción = U$D " + string(costoCalefaccion), 0, 0);
    xstring(0.1, 0.50, "Costo Refrigeración = U$D " + string(costoRefrigeracion), 0, 0);
    xstring(0.1, 0.25, "COSTO TOTAL = U$D " + string(costoTotal), 0, 0); 
endfunction

// -----------------------------------------------------------------------------
// Función: funcion_costo_climatizacion
// Descripción: Simula el día completo y calcula el costo monetario mensual.
//              Si 'graficar' es %T, invoca a grafico_salida.
// -----------------------------------------------------------------------------
function costoClimatizacion = funcion_costo_climatizacion(X, graficar)
    // Desempaquetado de variables de optimización
    hr_ini_cal = X(1);
    hr_cal     = X(2);
    hr_ini_ref = X(3);
    hr_ref     = X(4);
    
    // -- Paso A: Método de Euler (Simulación Temporal) --
    T_ini = 19.5;
    T = [T_ini];
    Dt = 36; // Paso de tiempo en segundos
    t = [0];
    N = floor((24 * 3600)/ Dt);
    
    for i = 1:N
        tc = t(i);
        k1 = f(tc, T(i), hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);
        T_next = T(i) + Dt * k1;
        T = [T, T_next];
        t = [t, tc + Dt];
    end
    
    // -- Paso B: Reconstrucción de perfiles de Potencia --
    Qc = [];
    Qr = [];
    for i = 1:length(t)
        Qc(i) = Q_calef(t(i), hr_ini_cal, hr_cal);
        Qr_val = Q_refri(t(i), hr_ini_ref, hr_ref);
        Qr(i) = abs(Qr_val); // Usamos valor absoluto para integrar energía
    end
    
    // -- Paso C: Cálculo de Energías y Costos --
    energiaCalefaccionDiaria = funcion_integral(t, Qc); // [J]
    energiaRefrigeracionDiaria = funcion_integral(t, Qr); // [J]
    
    // Conversión a Wh mensual (30 días)
    energiaCalefaccionMensual_Wh = energiaCalefaccionDiaria * 30 / 3600;
    costoCalefaccion = precioEnergiaCalefaccion * energiaCalefaccionMensual_Wh;
    
    energiaRefrigeracionMensual_Wh = energiaRefrigeracionDiaria * 30 / 3600;
    costoRefrigeracion = precioEnergiaRefrigeracion * energiaRefrigeracionMensual_Wh;
    
    costoClimatizacion = costoCalefaccion + costoRefrigeracion;
    
    // -- Salida en Consola --
    disp("--- Resultados intermedios ---");
    disp("Costo mensual calefacción (USD): " + string(costoCalefaccion));
    disp("Costo mensual refrigeración (USD): " + string(costoRefrigeracion));
    disp("Costo mensual TOTAL (USD): " + string(costoClimatizacion));

    // -- Gráfico opcional --
    if graficar then
        grafico_salida(t, T, Qc, Qr, costoRefrigeracion, costoCalefaccion, costoClimatizacion);
    end
endfunction

// Función auxiliar para obtener solo perfil T (usada por la función objetivo)
function temperatura = funcion_perfil_temperatura(X)
    hr_ini_cal = X(1); hr_cal = X(2); hr_ini_ref = X(3); hr_ref = X(4);
    
    T_ini = 19.5;
    T = [T_ini];
    Dt = 36;
    t = [0];
    N = floor((24 * 3600)/ Dt);
    
    for i = 1:N
        tc = t(i);
        k1 = f(tc, T(i), hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);
        T_next = T(i) + Dt * k1;
        T = [T, T_next];
        t = [t, tc + Dt];
    end
    temperatura = T;
endfunction


// =============================================================================
// 4. CONFIGURACIÓN DE LA OPTIMIZACIÓN
// =============================================================================

// Vector inicial [InicioCalefaccion, DuracionCalef, InicioRefri, DuracionRefri]
X = [0; 10; 12; 6];

// -----------------------------------------------------------------------------
// Función Objetivo (fobj)
// Incluye penalizaciones por discontinuidad térmica y varianza
// -----------------------------------------------------------------------------
function fcc = fobj(X)
    epsilon1 = 10;  // Penalización: Diferencia T final vs inicial
    epsilon2 = 100; // Penalización: Varianza (estabilidad térmica)
    epsilon3 = 0.1; // Penalización: Evitar división por cero (Calef)
    epsilon4 = 0.1; // Penalización: Evitar división por cero (Refri)
    
    temperatura_diaria = funcion_perfil_temperatura(X);
    diferencia_cuad_inicio_fin = (temperatura_diaria($) - temperatura_diaria(1))^2;
    varianza_temperatura = stdev(temperatura_diaria)^2;
    
    fcc = funcion_costo_climatizacion(X, %F) ...
        + epsilon1 * diferencia_cuad_inicio_fin ...
        + epsilon2 * varianza_temperatura ...
        + epsilon3 * 1/X(2) ...
        + epsilon4 * 1/X(4);
endfunction

// -----------------------------------------------------------------------------
// Derivadas Parciales Numéricas (Diferencias Centradas)
// -----------------------------------------------------------------------------
function dfx1 = Dfx1(X)
    h = 1e-2; e = [1;0;0;0];
    dfx1 = (fobj(X + h*e) - fobj(X - h*e)) / (2*h);
endfunction

function dfx2 = Dfx2(X)
    h = 1e-2; e = [0;1;0;0];
    dfx2 = (fobj(X + h*e) - fobj(X - h*e)) / (2*h);
endfunction

function dfx3 = Dfx3(X)
    h = 1e-2; e = [0;0;1;0];
    dfx3 = (fobj(X + h*e) - fobj(X - h*e)) / (2*h);
endfunction

function dfx4 = Dfx4(X)
    h = 1e-2; e = [0;0;0;1];
    dfx4 = (fobj(X + h*e) - fobj(X - h*e)) / (2*h);
endfunction

// Función Gradiente
function g = grad_f(X)
    d1 = Dfx1(X);
    d2 = Dfx2(X);
    d3 = Dfx3(X);
    d4 = Dfx4(X);
    g = [d1; d2; d3; d4];
endfunction


// =============================================================================
// 5. EJECUCIÓN PRINCIPAL (SIMULACIÓN FINAL)
// =============================================================================

// --- FASE 1: Simulación SIN Optimización ---
disp("=====================================================");
disp("  SIMULACIÓN SIN OPTIMIZACIÓN (PERFIL INICIAL)");
disp("=====================================================");

X_sin_opt = [0; 10; 12; 6];   
graficar = %T;
// Calculamos y guardamos costo inicial
Costo_Sin_Opt = funcion_costo_climatizacion(X_sin_opt, graficar);


// --- FASE 2: Proceso de Optimización (Descenso de Gradiente) ---
alpha = 0.01;    // Tasa de aprendizaje
max_iter = 50;   // Iteraciones máximas
tol = 1e-3;      // Tolerancia
X_opt = X_sin_opt;

disp("Iniciando optimización...");

for k = 1:max_iter
    g = grad_f(X_opt);
    
    if norm(g) < tol then
        break
    end
    
    X_opt = X_opt - alpha * g;

    // Proyecciones (Mantener variables en rangos físicos válidos)
    X_opt(1) = min(max(X_opt(1), 0), 24);   // Hora inicio Calef
    X_opt(2) = max(X_opt(2), 0.1);          // Duración Calef (>0)
    X_opt(3) = min(max(X_opt(3), 0), 24);   // Hora inicio Refri
    X_opt(4) = max(X_opt(4), 0.1);          // Duración Refri (>0)
end

printf("\nMínimo aproximado encontrado en X = [%f, %f, %f, %f]\n", X_opt(1), X_opt(2), X_opt(3), X_opt(4));


// --- FASE 3: Simulación CON Optimización ---
disp("=====================================================");
disp("  SIMULACIÓN CON OPTIMIZACIÓN (PERFIL ÓPTIMO)");
disp("=====================================================");

graficar = %T;
// Calculamos y guardamos costo optimizado
Costo_Con_Opt = funcion_costo_climatizacion(X_opt, graficar);


// --- FASE 4: Resumen Comparativo Final ---
disp(" ");
disp("************* RESUMEN COMPARATIVO *************");
disp("Costo Total SIN Optimizar: U$D " + string(Costo_Sin_Opt));
disp("Costo Total CON Optimización: U$D " + string(Costo_Con_Opt));
disp("AHORRO MENSUAL ESTIMADO:   U$D " + string(Costo_Sin_Opt - Costo_Con_Opt));
disp("***********************************************");
