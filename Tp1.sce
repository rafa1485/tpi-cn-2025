clear
clc


// DEFINICION PARA SALIDA GRAFICA
function grafico_salida(t,T,Qc,Qr,costoRefrigeracion,costoCalefaccion)
    figure()
    subplot(4,1,1)
    plot(t/3600,T)
    xtitle("Temperatura Interna del Edificio vs. Tiempo (h)", "Tiempo (h)", "Temperatura (°C)")
    
    subplot(4,1,2)
    plot(t/3600, Qc)
    xtitle("Potencia de Calefacción (Qc) vs. Tiempo (h)", "Tiempo (h)", "Potencia (W)")
    
    subplot(4,1,3)
    plot(t/3600, Qr)
    // Se usa plot(t/3600, -Qr) si quieres ver la potencia extraida como positiva
    xtitle("Potencia de Refrigeración (Qr) vs. Tiempo (h)", "Tiempo (h)", "Potencia (W)")
    
    subplot(4,1,4)
    xstring(0.1,0.33,"Costo Refrigeración Mensual = U$D"+string(costoRefrigeracion),0,0)
    xstring(0.1,0.66,"Costo Calefacción Mensual = U$D"+string(costoCalefaccion),0,0)
    xstring(0.1,0.99,"Costo Total Mensual = U$D"+string(costoRefrigeracion+costoCalefaccion),0,0)
endfunction

// --- PARÁMETROS AMBIENTALES Y DE TIEMPO (Página 2) ---
TAmbMax = 24 //"Máxima Temperatura Ambiente" [cite: 36]
TAmbMin = 10 //"Mínima Temperatura Ambiente" [cite: 36]
InicioSubida = 6 //"Hora en la que empieza a incrementar la temperatura" [cite: 37]
FinSubida = 11 //"Hora en la que empieza a incrementar la temperatura" [cite: 37]
InicioBajada = 14 //"Hora en la que empieza a decrementar la temperatura" [cite: 37]
FinBajada = 19 //"Hora en la que empieza a decrementar la temperatura" [cite: 37]


// --- PARÁMETROS DE LA CONSTRUCCIÓN Y COSTOS (Página 4) ---
superficieEdificacion=100 // [m2] [cite: 83]
superficiePiso=70 // [m2] [cite: 84]

espesorEdificacion = 0.3 // [m] [cite: 85]
coeficienteConductanciaEdificacion = 0.4 / espesorEdificacion // [W/K/m2] [cite: 86]
conductanciaEdificacion = coeficienteConductanciaEdificacion * superficieEdificacion // [W/K] [cite: 89]

espesorAislacionPiso = 0.05 // [m] [cite: 90]
coeficienteConductanciaPiso = 0.02 / espesorAislacionPiso // [W/K/m2] [cite: 91]
conductanciaPiso = superficiePiso*coeficienteConductanciaPiso // [W/K] [cite: 92]

potenciaCalefaccionUnitaria = 10 // Potencia de calefacción por metro cuadrado de superficie construida [W/m2] [cite: 93]
potenciaCalefaccion = potenciaCalefaccionUnitaria * superficiePiso // [W] [cite: 97]
precioEnergiaCalefaccion = 0.045/1000 // [dólares/Wh] [cite: 96] (Se usa el valor del TP, no el pre-escrito en el archivo .sce)

potenciaRefrigeracionUnitaria = 5 // Potencia de refrigeración por metro cuadrado de superficie construida [W/m2] [cite: 98, 99]
potenciaRefrigeracion = potenciaRefrigeracionUnitaria * superficiePiso // [W] [cite: 102]
precioEnergiaRefrigeracion = 0.12/1000 // [dólares/Wh] [cite: 103]

masaUnitaria = 150 // Masa de edificio por unidad de superficie de construcción [kg/m2] [cite: 104]
capacidadCalorificaEspecifica = 800 // Capacidad Calorífica por kg del material de construcción [J/kg/K] [cite: 105]
capacidadCalorificaUnitaria = masaUnitaria * capacidadCalorificaEspecifica // [J/K/m2] [cite: 106, 108, 109]
capacidadCalorificaEdificio = capacidadCalorificaUnitaria * superficiePiso // [J/K] [cite: 110]

// --- ESTIMACIÓN DEL COEFICIENTE 'h' (Página 2-3) ---
// Se estima h=18 W/(m2*C) para una velocidad de 3 m/s
h = 18 // coeficiente de transferencia de calor por convección [W/(m2*K)] [cite: 40]
conductanciaConveccionEdificacion = h * superficieEdificacion;


// --- FUNCIONES DE TRANSFERENCIA DE CALOR Y TEMPERATURA EXTERIOR ---
function T_ext = T_exterior(t)
    // t está en segundos.
    hr = t/3600; // t en horas
    if hr <= InicioSubida then
        T_ext = TAmbMin;
    elseif hr <= FinSubida then
        T_ext = ((TAmbMax - TAmbMin)/(FinSubida - InicioSubida))*(hr - InicioSubida) + TAmbMin;
    elseif hr <= InicioBajada then
        T_ext = TAmbMax;
    elseif hr <= FinBajada then
        T_ext = ((TAmbMin - TAmbMax)/(FinBajada - InicioBajada))*(hr - InicioBajada) + TAmbMax;
    else
        T_ext = TAmbMin;
    end
endfunction

function Qp = Q_piso(T_int)
    // Temperatura del suelo es 15 °C [cite: 77]
    Qp = conductanciaPiso * (15 - T_int);
endfunction

function Qe = Q_edif(t, T_int)
    T_ext = T_exterior(t) 
    Re = 1/conductanciaEdificacion; // Resistencia a la conducción
    Rc = 1/conductanciaConveccionEdificacion; // Resistencia a la convección
    // Conductancias en serie (resistencias se suman)
    conductanciaTotalEdificacion = 1/(Re + Rc); 
    Qe = conductanciaTotalEdificacion * (T_ext - T_int)
endfunction

function Qc = Q_calef(t,hr_ini_cal,hr_cal)
    hr_fin_cal = hr_ini_cal + hr_cal
    hr = t/3600;
    // La calefacción se aplica en el intervalo [hr_ini_cal, hr_fin_cal]
    if hr >= hr_ini_cal && hr <= hr_fin_cal then
        Qc = potenciaCalefaccion;
    else
        Qc = 0;
    end
endfunction

function Qr = Q_refri(t,hr_ini_ref,hr_ref)
    hr_fin_ref = hr_ini_ref + hr_ref
    hr = t/3600;
    // La refrigeración se aplica en el intervalo [hr_ini_ref, hr_fin_ref]
    if hr >= hr_ini_ref && hr <= hr_fin_ref then
        Qr = -potenciaRefrigeracion; // Flujo de calor negativo (extracción)
    else
        Qr = 0;
    end
endfunction


function Qt = Q_total(t, T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    Qp = Q_piso(T_int); 
    Qe = Q_edif(t,T_int); 
    Qc = Q_calef(t,hr_ini_cal,hr_cal); 
    Qr = Q_refri(t,hr_ini_ref,hr_ref); 
    Qt = Qp + Qe + Qc + Qr; // Suma algebraica de flujos
endfunction

function dT = f(t,T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    // EDO: d(T_int)/dt = Q_total / C_edificio
    dT = Q_total(t,T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref) / capacidadCalorificaEdificio;
endfunction


function costoClimatizacion = funcion_costo_climatizacion(X, graficar)

    
    // DEFINICION VARIABLES DE CONTROL TEMPERATURA
    hr_ini_cal = X(1) // Hora de inicio de calefacción (h)
    hr_cal = X(2) // Duración de calefacción (h)
    hr_ini_ref = X(3) // Hora de inicio de refrigeración (h)
    hr_ref = X(4) // Duración de refrigeración (h)
    
    
    // PROGRAMACION METOD0 DE EULER PARA CALCULAR LA EVOLUCION DE LA TEMPERATURA
    T_ini = 19.5; // T inicial en el rango [18, 22] °C [cite: 112]
    T = [T_ini]
    Dt = 36; // Paso de tiempo en segundos (0.01 h)
    t = [0]
    N = (24 * 3600)/ Dt; // Número de pasos
    
    // **************** IMPLEMENTACIÓN DE EULER ****************
    for i=1:N,
        // Calcular la variación de temperatura dT/dt
        dT_i = f(t(i), T(i), hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);

        // Aplicar el método de Euler: T(i+1) = T(i) + Dt * f(t(i), T(i))
        T_new = T(i) + Dt * dT_i;

        // Actualizar los vectores
        T = [T, T_new];
        t = [t, t(i) + Dt];
    end
    
    
    // CALCULO DEL PERFIL DE CALOR DE CALEFACCION Y REFRIGERACION
    Qc = []
    Qr = []
    
    for i=1:N+1,
        // Q_calef y Q_refri devuelven la potencia en Watts (J/s)
        Qc = [Qc, Q_calef(t(i), hr_ini_cal, hr_cal)];
        Qr = [Qr, Q_refri(t(i), hr_ini_ref, hr_ref)]
    end

    
    // INTEGRACION DE LA ENERGIA (SUMA DE RIEMANN)
    // Energía [Joules] = Sumatoria( Potencia [W] * Delta_t [s] )
    
    energiaCalefaccionDiaria = 0
    energiaRefrigeracionDiaria = 0 // [Joules]

    // **************** IMPLEMENTACIÓN DE INTEGRAL (SUMA DE RIEMANN) ****************
    for i=1:N, // El bucle va de 1 a N, usando los valores de Qc/Qr calculados.
        energiaCalefaccionDiaria = energiaCalefaccionDiaria + Qc(i) * Dt;
        energiaRefrigeracionDiaria = energiaRefrigeracionDiaria + abs(Qr(i)) * Dt;
    end
    
    // Conversión a Wh y Costo Mensual
    // 1 Wh = 3600 J
    energiaCalefaccionMensual_Wh = energiaCalefaccionDiaria * 30 / 3600
    costoCalefaccion = precioEnergiaCalefaccion * energiaCalefaccionMensual_Wh
    
    energiaRefrigeracionMensual_Wh = energiaRefrigeracionDiaria * 30 / 3600
    costoRefrigeracion = precioEnergiaRefrigeracion * energiaRefrigeracionMensual_Wh
    
    costoClimatizacion = costoCalefaccion + costoRefrigeracion

 
    if graficar then
        grafico_salida(t,T,Qc,Qr,costoRefrigeracion,costoCalefaccion)
    end
    
    // Devolvemos el CostoTotal real y el Perfil de Temperatura
    costoClimatizacion = [costoClimatizacion, T];
endfunction


function temperatura = funcion_perfil_temperatura(X)
    // Esta función se usa para obtener solo el perfil de temperatura.
    hr_ini_cal = X(1)
    hr_cal = X(2)
    hr_ini_ref = X(3)
    hr_ref = X(4)
    
    T_ini = 19.5;
    T = [T_ini]
    Dt = 36;
    t = [0]
    N = (24 * 3600)/ Dt;
    
    // **************** IMPLEMENTACIÓN DE EULER ****************
    for i=1:N,
        dT_i = f(t(i), T(i), hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);
        T_new = T(i) + Dt * dT_i;
        T = [T, T_new];
        t = [t, t(i) + Dt];
    end
    
    temperatura = T
endfunction

// --- OPTIMIZACIÓN CON GRADIENTE DESCENDENTE ---

// Valores iniciales (se pueden modificar para explorar diferentes puntos de partida)
inicioCalefaccion = 0.5 
duracionCalefaccion = 15 
inicioRefrigeracion = 12
duracionRefrigeracion = 1 

// Vector de control inicial: X = [hr_ini_cal; hr_cal; hr_ini_ref; hr_ref]
X = [inicioCalefaccion;
     duracionCalefaccion;
     inicioRefrigeracion;
     duracionRefrigeracion]
     
graficar = %T 
printf("======================================================\n")
printf("1. SIMULACIÓN INICIAL (SIN OPTIMIZAR)\n")
printf("Vector de control inicial X = [%f, %f, %f, %f]\n", X(1), X(2), X(3), X(4))
funcion_costo_climatizacion(X, graficar);

// Paso para la derivada numérica
h = 1e-4

function fcc = fobj(X)
    // Coeficientes de penalización
    epsilon1 = 5000 // Penalización para T_final - T_inicial (cierre de ciclo)
    epsilon2 = 2000 // Penalización para Varianza (estabilidad y cumplimiento de rango)
    epsilon3 = 0.1 // Penalización para asegurar duración de calefacción positiva
    epsilon4 = 0.1 // Penalización para asegurar duración de refrigeración positiva
    
    T_sim = funcion_costo_climatizacion(X, %F) // T_sim(1) es el Costo Total
    f_costo = T_sim(1) // Costo real mensual

    temperatura_diaria = T_sim(2:$)
    
    // Penalización 1: T_final - T_inicial (para que el ciclo se mantenga, T_final=T($))
    diferencia_cuad_inicio_fin = (temperatura_diaria($) - temperatura_diaria(1))^2
    
    // Penalización 2: Varianza (minimizar fluctuaciones)
    varianza_temperatura = stdev(temperatura_diaria)^2
    
    // Función objetivo = Costo real + Penalizaciones
    fcc = f_costo + epsilon1 * diferencia_cuad_inicio_fin + epsilon2 * varianza_temperatura + epsilon3*1/X(2) + epsilon4*1/X(4)
endfunction

// DEFINICION DE DERIVADAS PARCIALES NUMERICAS

function dfx1 = Dfx1(X)
    X_h = X;
    X_h(1) = X(1) + h;
    dfx1 = (fobj(X_h) - fobj(X)) / h;
endfunction

function dfx2 = Dfx2(X)
    X_h = X;
    X_h(2) = X(2) + h;
    dfx2 = (fobj(X_h) - fobj(X)) / h; 
endfunction

function dfx3 = Dfx3(X)
    X_h = X;
    X_h(3) = X(3) + h;
    dfx3 = (fobj(X_h) - fobj(X)) / h; 
endfunction

function dfx4 = Dfx4(X)
    X_h = X;
    X_h(4) = X(4) + h;
    dfx4 = (fobj(X_h) - fobj(X)) / h; 
endfunction

// DEFINICION DE LA FUNCIÓN GRADIENTE
function g = grad_f(X)
    d1 = Dfx1(X)
    d2 = Dfx2(X)
    d3 = Dfx3(X)
    d4 = Dfx4(X)
    g = [d1;
         d2; d3; d4]
endfunction



// GRADIENTE DESCENDENTE
alpha = 0.005 // Tasa de aprendizaje
max_iter = 50 // Máximo número de iteraciones
tol = 0.01 // Tolerancia

printf("\n======================================================\n")
printf("2. OPTIMIZACIÓN CON GRADIENTE DESCENDENTE\n")
printf("Iniciando optimización (alpha=%f, max_iter=%d)...\n", alpha, max_iter)


for k = 1:max_iter
    
    g = grad_f(X); // Calcular el gradiente
    norma_g = norm(g); // Norma del gradiente
    
    if norma_g < tol then
        printf("Convergencia alcanzada en la iteración %d (Norma Gradiente: %f).\n", k, norma_g);
        break
    end
    
    // Actualización de X: X_k+1 = X_k - alpha * grad(f)
    X_new = X - alpha * g;
    
    // Restricciones y límites
    
    // Horas de inicio (X(1) y X(3)): ciclo diario [0, 24)
    X_new(1) = modulo(X_new(1), 24); 
    X_new(3) = modulo(X_new(3), 24); 
    
    // Duraciones (X(2) y X(4)): deben ser positivas y menores a 24.
    if X_new(2) <= 0 then
        X_new(2) = 0.01
    elseif X_new(2) > 24 then
        X_new(2) = 24
    end
    
    if X_new(4) <= 0 then
        X_new(4) = 0.01
    elseif X_new(4) > 24 then
        X_new(4) = 24
    end

    X = X_new;
    
    if k == max_iter then
        printf("Máximo de iteraciones (%d) alcanzado.\n", max_iter);
    end
end

// Presentar nueva solución
printf("\n======================================================\n")
printf("3. RESULTADO OPTIMIZADO\n")
printf("Vector de control óptimo X = [hr_ini_cal: %f, hr_cal: %f, hr_ini_ref: %f, hr_ref: %f]\n", X(1), X(2), X(3), X(4));
graficar = %T 
printf("Costo mínimo de climatización (mensual):\n")
funcion_costo_climatizacion(X, graficar);