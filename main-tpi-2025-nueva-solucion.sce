clear
clc


// DEFINICION PARA SALIDA GRAFICA
function grafico_salida(t,T,Qc,Qr,costoRefrigeracion,costoCalefaccion)
    figure()
    subplot(4,1,1)
    plot(t/3600,T)
    
    subplot(4,1,2)
    plot(t/3600, Qc,'r')
    
    subplot(4,1,3)
    plot(t/3600, Qr,'b')
    
    subplot(4,1,4)
    xstring(0.1,0.33,"Costo Refrigeración= U$D"+string(costoRefrigeracion),0,0)
    xstring(0.1,0.66,"Costo Calefacción= U$D"+string(costoCalefaccion),0,0)
endfunction

TAmbMax = 24 //"Máxima Temperatura Ambiente"
TAmbMin = 10 //"Mínima Temperatura Ambiente"
InicioSubida = 6 //"Hora en la que empieza a incrementar la temperatura"
FinSubida = 11 //"Hora en la que empieza a incrementar la temperatura"
InicioBajada = 14 //"Hora en la que empieza a decrementar la temperatura"
FinBajada = 19 //"Hora en la que empieza a decrementar la temperatura"


superficieEdificacion=100 // [m2]
superficiePiso=70 // [m2]

espesorEdificacion = 0.3 // [m]
coeficienteConductanciaEdificacion = 0.4 / espesorEdificacion // [W/K/m2]
conductanciaEdificacion = coeficienteConductanciaEdificacion * superficieEdificacion // [W/K]

espesorAislacionPiso = 0.05 // [m]
coeficienteConductanciaPiso = 0.02 / espesorAislacionPiso // [W/K/m2]
conductanciaPiso = superficiePiso*coeficienteConductanciaPiso // [W/K]

potenciaCalefaccionUnitaria = 10 // Potencia de calefacción por metro cuadrado de superficie construida [W/m2]
potenciaCalefaccion = potenciaCalefaccionUnitaria * superficiePiso // [W]
precioEnergiaCalefaccion = 0.045/1000 // [dólares/Wh]

// CALCULO DEL COSTO DE LA ENERGIA DE CALEFACCION
//poderCalorificoGas = 10.8 //[kWh/m3]
//precioM3Gas = 180 // [$/m3]
//precio_energia_Gas_Pesos_kWh = precioM3Gas / poderCalorificoGas / 0.8 // Considerando un rendimiento termico de 0.8
//precioDolar_Pesos = 1500 
//precio_energia_Gas_USD_kWh = precio_energia_Gas_Pesos_kWh / precioDolar_Pesos
//precio_energia_Gas_USD_Wh = precio_energia_Gas_USD_kWh / 1000

potenciaRefrigeracionUnitaria = 5 // Potencia de refrigeración por metro cuadrado de superficie construida [W/m2]
potenciaRefrigeracion = potenciaRefrigeracionUnitaria * superficiePiso // [W]
precioEnergiaRefrigeracion = 0.12/1000 // [dólares/Wh]

masaUnitaria = 150 // Masa de edificio por unidad de superficie de construcción [kg/m2]
capacidadCalorificaEspecifica = 800 // Capacidad Calorífica por kg del material de construcción [J/kg/K]
capacidadCalorificaUnitaria = masaUnitaria * capacidadCalorificaEspecifica // [J/K/m2]
capacidadCalorificaEdificio = capacidadCalorificaUnitaria * superficiePiso // [J/K]

h = 18 // coeficiente de transferencia de calor por convección de la edificación a la velocidad de 3 m/s del aire
conductanciaConveccionEdificacion = h * superficieEdificacion;

function T_ext = T_exterior(t)
    if t <= InicioSubida*3600 then
        T_ext = TAmbMin;
    elseif t <= FinSubida*3600 then
        T_ext = ((TAmbMax - TAmbMin)/(FinSubida - InicioSubida))*(t/3600 - InicioSubida) + TAmbMin;
    elseif t <= InicioBajada*3600 then
        T_ext = TAmbMax;
    elseif t <= FinBajada*3600 then
        T_ext = ((TAmbMin - TAmbMax)/(FinBajada - InicioBajada))*(t/3600 - InicioBajada) + TAmbMax;
    else
        T_ext = TAmbMin;
    end
endfunction

function Qp = Q_piso(T_int)
    Qp = conductanciaPiso * (15 - T_int);
endfunction

function Qe = Q_edif(t, T_int)
    T_ext = T_exterior(t) 
    Re = 1/conductanciaEdificacion; // Resistencia a la transferencia de calor por la pared de la edificación
    Rc = 1/conductanciaConveccionEdificacion; // Resistencia a la transferencia de calor por convección en la edificación
    conductanciaTotalEdificacion = 1/(Re + Rc);
    Qe = conductanciaTotalEdificacion * (T_ext - T_int)
endfunction

function Qc = Q_calef(t,hr_ini_cal,hr_cal)
    hr_fin_cal = hr_ini_cal + hr_cal
    if (t/3600) >= hr_ini_cal && (t/3600) <= hr_fin_cal then
         if (t/3600 > 8 && t/3600 < 15) then
            Qc = potenciaCalefaccion;// * 0.7;
        else
            Qc = potenciaCalefaccion;
        end
    else
        Qc = 0;
    end
endfunction

function Qr = Q_refri(t,hr_ini_ref,hr_ref)
    hr_fin_ref = hr_ini_ref + hr_ref
    if t/3600 >= hr_ini_ref && t/3600 <= hr_fin_ref then
        Qr = potenciaRefrigeracion;
    else
        Qr = 0;
    end
endfunction


function Qt = Q_total(t, T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    Qp = Q_piso(T_int);
    Qe = Q_edif(t,T_int);
    Qc = Q_calef(t,hr_ini_cal,hr_cal)
    Qr = Q_refri(t,hr_ini_ref,hr_ref)
    Qt = Qp + Qe + Qc - Qr;
endfunction

function dT = f(t,T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    dT = Q_total(t,T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref) / capacidadCalorificaEdificio;
endfunction

function integral = funcion_integral(t, Q)
    integral = 0  // Inicializamos el acumulador de la integral
    n = length(t)  // Obtenemos el número de puntos en el vector tiempo
    
    // Recorremos todos los intervalos entre puntos consecutivos
    for i = 1:(n-1)
        dt = t(i+1) - t(i)  // Calculamos el ancho del intervalo (base del trapecio)
        
        // Aplicamos la regla del trapecio: Área = (base * (altura1 + altura2)) / 2
        // Q(i) y Q(i+1) son las alturas en los extremos del trapecio
        integral = integral + (Q(i) + Q(i+1)) * dt / 2
    end
endfunction

function costoClimatizacion = funcion_costo_climatizacion(X, graficar)

    
    // DEFINICION VARIABLES DE CONTROL TEMPERATURA
    hr_ini_cal = X(1)
    hr_cal = X(2)
    hr_ini_ref = X(3)
    hr_ref = X(4)
    
    
    // PROGRAMACION METOD0 DE EULER PARA CALCULAR LA EVOLUCION DE LA TEMPERATURA
    T_ini = 19.5;
    T = [T_ini]
    T_ext = [T_exterior(0)]
    Dt = 36;
    t = [0]
    N = (24 * 3600)/ Dt;
    
    
    for i=1:N,
        // METODO DE EULER
        tiempo_actual = t(i)  // Tomamos el tiempo actual del vector
        Temperatura_actual = T(i)  // Tomamos la temperatura actual del vector
        
        // Calculamos la derivada dT/dt en el punto actual
        // Esta derivada representa la tasa de cambio de temperatura
        // La function f es la dT mas arriba
        dT = f(tiempo_actual, Temperatura_actual, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
        
        // Aplicamos la fórmula de Euler: T(i+1) = T(i) + f(t,T) * Δt
        // Esto es como decir: nueva_temperatura = temperatura_actual + cambio_en_Dt_segundos
        Temperatura_nueva = Temperatura_actual + dT * Dt
        
        // Avanzamos el tiempo sumando el paso temporal Dt
        tiempo_nuevo = tiempo_actual + Dt
        
        // Agregamos los nuevos valores a los vectores para seguir iterando
        T = [T, Temperatura_nueva]
        t = [t, tiempo_nuevo]
    end
    
    // CALCULO DEL PERFIL DE CALOR DE CALEFACCION Y REFRIGERACION
    Qc = [Q_calef(0)]
    Qr = [Q_refri(0)]
    
    for i=1:N,
        Qc = [Qc, Q_calef(t(i), hr_ini_cal, hr_cal)];
        Qr = [Qr, Q_refri(t(i), hr_ini_ref, hr_ref)]
    end

    
    // INTEGRACION DE LA ENERGIA DE CALEFACCION A LO LARGO DEL DIA (JOULES)
    energiaCalefaccionDiaria = funcion_integral(t, Qc); // [Joules]
    
    // INTEGRACION DE LA ENERGIA DE REFRIGERACION A LO LARGO DEL DIA (JOULES)
    energiaRefrigeracionDiaria = funcion_integral(t, Qr); // [Joules]
    
    energiaCalefaccionMensual_Wh = energiaCalefaccionDiaria * 30 / 3600
    
    costoCalefaccion = precioEnergiaCalefaccion * energiaCalefaccionMensual_Wh
    
    energiaRefrigeracionMensual_Wh = energiaRefrigeracionDiaria * 30 / 3600
    
    costoRefrigeracion = precioEnergiaRefrigeracion * energiaRefrigeracionMensual_Wh
    
    costoClimatizacion = costoCalefaccion + costoRefrigeracion
    
    disp(costoClimatizacion)

    if graficar then
        grafico_salida(t,T,Qc,Qr,costoRefrigeracion,costoCalefaccion)
    end
endfunction


function temperatura = funcion_perfil_temperatura(X)
    // DEFINICION VARIABLES DE CONTROL TEMPERATURA
    hr_ini_cal = X(1)
    hr_cal = X(2)
    hr_ini_ref = X(3)
    hr_ref = X(4)
    
    // PROGRAMACION METOD0 DE EULER PARA CALCULAR LA EVOLUCION DE LA TEMPERATURA
    T_ini = 19.5;
    T = [T_ini]
    T_ext = [T_exterior(0)]
    Dt = 36;
    t = [0]
    N = (24 * 3600)/ Dt;
    
    for i=1:N,
        tiempo_actual = t(i)  // Tomamos el tiempo actual del vector
        Temperatura_actual = T(i)  // Tomamos la temperatura actual del vector
        
        // Calculamos la derivada dT/dt en el punto actual
        // Esta derivada representa la tasa de cambio de temperatura
        // La function f es la dT mas arriba
        dT = f(tiempo_actual, Temperatura_actual, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
        
        // Aplicamos la fórmula de Euler: T(i+1) = T(i) + f(t,T) * Δt
        // Esto es como decir: nueva_temperatura = temperatura_actual + cambio_en_Dt_segundos
        Temperatura_nueva = Temperatura_actual + dT * Dt
        
        // Avanzamos el tiempo sumando el paso temporal Dt
        tiempo_nuevo = tiempo_actual + Dt
        
        // Agregamos los nuevos valores a los vectores para seguir iterando
        T = [T, Temperatura_nueva]
        t = [t, tiempo_nuevo]
    
    end
    
    temperatura = T
endfunction

// PROGAMACION OPTIMIZACIÓN
inicioCalefaccion = 0 // "Hora a la que se enciende la Refrigeracion"
finCalefaccion = 10 // "Hora a la que se apaga la refrigeración"
inicioRefrigeracion = 13 // "Hora a la que se enciende la Refrigeracion"
finRefrigeracion = 4 // "Hora a la que se apaga la refrigeración"

X = [inicioCalefaccion;
     finCalefaccion;
     inicioRefrigeracion;
     finRefrigeracion]
     
graficar = %T // %T : graficar , %F : NO graficar
funcion_costo_climatizacion(X, graficar);

function fcc = fobj(X)
    // RESTRICCIONES DE RANGO (0 a 24 horas)
    // Si el algoritmo propone tiempos negativos o mayores a 24 (para inicio), devolvemos infinito.
    if or(X < 0) | or(X > 24) then
        fcc = %inf;
        return;
    end

    // OBTENCIÓN DE DATOS
    temperatura_diaria = funcion_perfil_temperatura(X);
    costo_dinero = funcion_costo_climatizacion(X, %F); // %F para no graficar

    // PENALIZACIÓN POR TEMPERATURA (Estricta 18°C - 22°C)
    // Reemplaza a la varianza. Castiga fuertemente si sale de la zona de confort.
    
    // Exceso por calor (> 22):
    diferencia_sup = temperatura_diaria - 22;
    violacion_sup = sum(diferencia_sup(diferencia_sup > 0).^2); 
    
    // Exceso por frío (< 18):
    diferencia_inf = 18 - temperatura_diaria;
    violacion_inf = sum(diferencia_inf(diferencia_inf > 0).^2);
    
    penalidad_temp = 7000 * (violacion_sup + violacion_inf);

    // PENALIZACIÓN POR CICLO
    // La temperatura final debe ser igual a la inicial para que el ciclo sea sostenible.
    diferencia_cuad_inicio_fin = (temperatura_diaria($) - temperatura_diaria(1))^2;
    penalidad_ciclo = 100 * diferencia_cuad_inicio_fin;

    // PENALIZACIÓN POR SOLAPAMIENTO
    // Evita que Calefacción y Refrigeración se prendan al mismo tiempo.
    ini_c = X(1); fin_c = X(1) + X(2);
    ini_r = X(3); fin_r = X(3) + X(4);
    
    // Cálculo de intersección de horarios
    horas_solapadas = max(0, min(fin_c, fin_r) - max(ini_c, ini_r));
    penalidad_overlap = 10000 * horas_solapadas;

    // PENALIZACIÓN POR DURACIÓN EXCESIVA (> 24h)
    // Arregla el "bug" donde la duración se iba a 25 horas.
    penalidad_duracion = 0;
    if X(2) > 24 then 
        penalidad_duracion = penalidad_duracion + 50000 * (X(2)-24)^2; 
    end
    if X(4) > 24 then 
        penalidad_duracion = penalidad_duracion + 50000 * (X(4)-24)^2; 
    end
    
    // PENALIZACIÓN POR DURACIÓN EXCESIVA
    // Factor_Duracion: un valor para castigar las horas de encendido (ej. 100)
    Factor_Duracion = 100; 

    // Castiga la duración linealmente: más horas encendidas = mayor costo de fobj
    penalidad_duracion_larga = Factor_Duracion * (X(2) + X(4));

    costo_escalado = costo_dinero * 100;
    // (Objetivo a Minimizar)
    fcc = costo_escalado + penalidad_temp + penalidad_ciclo + penalidad_overlap + penalidad_duracion + penalidad_duracion_larga;

endfunction

// Presentar nueva solución
// FUNCIÓN OBJETIVO ROBUSTA

function penalidad = calcular_penalidad_solapamiento(X)
    // Evitar que Calefacción y Refrigeración se prendan a la vez
    ini_cal = X(1); fin_cal = X(1) + X(2);
    ini_ref = X(3); fin_ref = X(3) + X(4);
    
    solapamiento = max(0, min(fin_cal, fin_ref) - max(ini_cal, ini_ref));
    
    if solapamiento > 0 then penalidad = 10000 * solapamiento; else penalidad = 0; end
endfunction

// 2. ALGORITMO DE RECOCIDO SIMULADO (SIMULATED ANNEALING)

printf("\n=== INICIANDO RECOCIDO SIMULADO ===\n");

// Configuración del Algoritmo
T = 100;           // Temperatura inicial (Alta para explorar)
T_min = 1;          // Temperatura final (Baja para refinar)
alpha = 0.95;       // Factor de enfriamiento (0.90 a 0.99)
iter_por_temp = 30; // Cuántos vecinos probar en cada nivel de temperatura

// Solución Inicial
// X = [InicioCal, DuracionCal, InicioRef, DuracionRef]
X_actual = [0; 12; 12; 4]; // Arrancamos con una lógica básica
costo_actual = fobj(X_actual);

X_mejor = X_actual;
costo_mejor = costo_actual;

// Bucle de Enfriamiento
contador_total = 0;

while T > T_min
    
    for i = 1:iter_por_temp
        // 1. Generar Vecino (Perturbación aleatoria)
        // Modificamos los tiempos en un rango de +/- 2 horas aprox
        ruido = (rand(4,1) - 0.5) * 4; 
        X_nuevo = X_actual + ruido;
        
        // 2. Clamping (Forzar límites 0-24)
        // Si se pasa de 24, lo dejamos en 24. Si es < 0, en 0.
        X_nuevo(X_nuevo > 24) = 24;
        X_nuevo(X_nuevo < 0) = 0;
        
        // 3. Evaluar Energía (Costo)
        costo_nuevo = fobj(X_nuevo);
        
        // 4. Calcular Delta de Energía
        delta_E = costo_nuevo - costo_actual;
         
        // 5. Criterio de Aceptación (Metrópolis)
        aceptar = %F;
        
        if delta_E < 0 then
            // Si es mejor, lo aceptamos siempre
            aceptar = %T;
        else
            // Si es peor, lo aceptamos con probabilidad P = exp(-delta / T)
            // Esto permite salir de mínimos locales al principio
            probabilidad = exp(-delta_E / T);
            if rand() < probabilidad then
                aceptar = %T;
            end
        end
        
        // 6. Actualizar estados
        if aceptar then
            X_actual = X_nuevo;
            costo_actual = costo_nuevo;
            
            // ¿Es el mejor histórico?
            if costo_actual < costo_mejor then
                X_mejor = X_actual;
                costo_mejor = costo_actual;
            end
        end
        
        contador_total = contador_total + 1;
    end
    
    // Reporte visual
    printf("Temp: %.2f | fobj: %.4f | X_mejor: [%.1f, %.1f, %.1f, %.1f]\n", T, costo_mejor, X_mejor(1), X_mejor(2), X_mejor(3), X_mejor(4));
    
    // Enfriar el sistema
    T = T * alpha;
end


// RESULTADO FINAL

printf("\n SOLUCIÓN ÓPTIMA (SIMULATED ANNEALING) \n");
printf("Iteraciones totales: %d\n", contador_total);
printf("Calefacción:   Inicio %5.2fh | Duración %5.2fh\n", X_mejor(1), X_mejor(2));
printf("Refrigeración: Inicio %5.2fh | Duración %5.2fh\n", X_mejor(3), X_mejor(4));

// Graficar solución final con el costo real
graficar = %T;
funcion_costo_climatizacion(X_mejor, graficar);