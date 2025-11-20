clear
clc


// DEFINICION PARA SALIDA GRAFICA
function grafico_salida(t,T,Qc,Qr,costoRefrigeracion,costoCalefaccion)
    figure()
    
    // 1. TEMPERATURA INTERNA
    subplot(4,1,1)
    plot(t/3600,T)
    title("Evolución de la Temperatura Interna")
    xlabel("Tiempo (h)")
    ylabel("Temperatura (°C)")
    
    // 2. POTENCIA DE CALEFACCIÓN
    subplot(4,1,2)
    plot(t/3600, Qc,'r')
    title("Potencia de Calefacción (Qc)")
    xlabel("Tiempo (h)")
    ylabel("Potencia (W)")
    
    // 3. POTENCIA DE REFRIGERACIÓN
    subplot(4,1,3)
    plot(t/3600, Qr,'b')
    title("Potencia de Refrigeración (Qr)")
    xlabel("Tiempo (h)")
    ylabel("Potencia (W)")
    
    // 4. COSTOS MENSUALES
    subplot(4,1,4)
    // Cálculo y muestra del Costo Total de Climatización
    costoTotal = costoRefrigeracion + costoCalefaccion;
    xstring(0.1,0.1,"Costo Total Climatización= U$D"+string(costoTotal),0,0) // Costo Total
    xstring(0.1,0.33,"Costo Refrigeración= U$D"+string(costoRefrigeracion),0,0)
    xstring(0.1,0.66,"Costo Calefacción= U$D"+string(costoCalefaccion),0,0)
    xlabel("Costos Mensuales (U$D)")
    ylabel(" ")
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
    
    // Condición corregida: se enciende a hr_ini_cal y se apaga a hr_fin_cal
    if (t/3600) >= hr_ini_cal && (t/3600) <= hr_fin_cal then
        Qc = potenciaCalefaccion;
    else
        Qc = 0;
    end
endfunction

function Qr = Q_refri(t,hr_ini_ref,hr_ref)
    hr_fin_ref = hr_ini_ref + hr_ref
    
    // Usamos el tiempo en horas para la condición de encendido/apagado
    if (t/3600) >= hr_ini_ref && (t/3600) <= hr_fin_ref then
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
    Qt = Qp + Qe + Qc + Qr;
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
        dT = f(tiempo_actual, Temperatura_actual, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
        
        // Aplicamos la fórmula de Euler: T(i+1) = T(i) + f(t,T) * Δt
        Temperatura_nueva = Temperatura_actual + dT * Dt
        
        // Avanzamos el tiempo sumando el paso temporal Dt
        tiempo_nuevo = tiempo_actual + Dt
        
        // Agregamos los nuevos valores a los vectores para seguir iterando
        T = [T, Temperatura_nueva]
        t = [t, tiempo_nuevo]
    end
    
    
    // CALCULO DEL PERFIL DE CALOR DE CALEFACCION Y REFRIGERACION
    // Recalculamos Qr y Qc usando el vector de tiempo completo
    Qc = []
    Qr = []
    
    for i=1:length(t),
        Qc = [Qc, Q_calef(t(i), hr_ini_cal, hr_cal)];
        Qr = [Qr, Q_refri(t(i), hr_ini_ref, hr_ref)]
    end

    
    // INTEGRACION DE LA ENERGIA DE CALEFACCION A LO LARGO DEL DIA (JOULES)
    energiaCalefaccionDiaria = funcion_integral(t, Qc)
    
    
    // INTEGRACION DE LA ENERGIA DE REFRIGERACION A LO LARGO DEL DIA (JOULES)
    energiaRefrigeracionDiaria = funcion_integral(t, Qr) // [Joules]
    
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
        // METODO DE EULER
        tiempo_actual = t(i)
        Temperatura_actual = T(i)
        
        dT = f(tiempo_actual, Temperatura_actual, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
        Temperatura_nueva = Temperatura_actual + dT * Dt
        tiempo_nuevo = tiempo_actual + Dt
        
        T = [T, Temperatura_nueva]
        t = [t, tiempo_nuevo]
    
    end
    
    temperatura = T
endfunction

// PROGAMACION OPTIMIZACIÓN CON GRADIETNE DESCENDENTE
inicioCalefaccion = 0 // "Hora a la que se enciende la Calefaccion"
finCalefaccion = 24 // "Duración de la Calefacción"
inicioRefrigeracion = 8 // "Hora a la que se enciende la Refrigeracion"
finRefrigeracion = 6 // "Duración de la Refrigeración"

X = [inicioCalefaccion;
     finCalefaccion;
     inicioRefrigeracion;
     finRefrigeracion]
     
graficar = %T // %T : graficar , %F : NO graficar
funcion_costo_climatizacion(X, graficar);

function fcc = fobj(X)
    // Penalización para las restricciones de optimización
    epsilon1 = 10
    epsilon2 = 100
    epsilon3 = 0.1
    epsilon4 = 0.1
    
    // Penalización 1: Diferencia al inicio y al final del día (ciclo constante)
    temperatura_diaria = funcion_perfil_temperatura(X)
    diferencia_cuad_inicio_fin = (temperatura_diaria($) - temperatura_diaria(1))^2
    
    // Penalización 2: Varianza (mantener la temperatura estable)
    varianza_temperatura = stdev(temperatura_diaria)^2
    
    // Término principal: Costo monetario
    // Término de penalización para evitar duración de 0 (X(2) y X(4) en el denominador)
    fcc = funcion_costo_climatizacion(X, %F) + epsilon1 * diferencia_cuad_inicio_fin + epsilon2 * varianza_temperatura + epsilon3*1/X(2) + epsilon4*1/X(4)
endfunction

// DEFINICION DE DERIVADAS PARCIALES NUMERICAS

function dfx1 = Dfx1(X)
    // Calculamos la derivada parcial respecto a hr_ini_cal (X(1))
    h = 1e-4 // Paso pequeño para la aproximación numérica (RECOMENDADO)
    X_h = X  // Copia de X para incrementar
    X_h(1) = X_h(1) + h  // Incrementamos solo la primera componente
    dfx1 = (fobj(X_h) - fobj(X)) / h
endfunction

function dfx2 = Dfx2(X)
    h = 1e-4
    X_h = X
    X_h(2) = X_h(2) + h  // Incrementamos solo la segunda componente
    dfx2 = (fobj(X_h) - fobj(X)) / h
endfunction

function dfx3 = Dfx3(X)
    h = 1e-4
    X_h = X
    X_h(3) = X_h(3) + h  // Incrementamos solo la tercera componente
    dfx3 = (fobj(X_h) - fobj(X)) / h
endfunction

function dfx4 = Dfx4(X)
    h = 1e-4
    X_h = X
    X_h(4) = X_h(4) + h  // Incrementamos solo la cuarta componente
    dfx4 = (fobj(X_h) - fobj(X)) / h
endfunction

// DEFINICION DE LA FUNCIÓN GRADIENTE
function g = grad_f(X)
    d1 = Dfx1(X)
    d2 = Dfx2(X)
    d3 = Dfx3(X)
    d4 = Dfx4(X)
    g = [d1; d2; d3; d4]
endfunction



// GRADIENTE DESCENDENTE
alpha = 0.01
max_iter = 100
tol = 1e-6 // <--- SOLUCIÓN: Tolerancia reducida para forzar la convergencia

for k = 1:max_iter
    // Calculamos el gradiente en el punto actual
    grad = grad_f(X)
    
    // Actualizamos X moviéndonos en dirección OPUESTA al gradiente
    X_anterior = X;
    X = X - alpha * grad
    
    // Verificamos si la magnitud del gradiente es menor que la tolerancia
    if norm(grad) < tol then
        printf("Convergencia alcanzada en iteración %d\n", k)
        break  // Salimos del bucle porque ya encontramos el mínimo
    end
    
    // Mostramos el progreso
    printf("Iteración %d: X = [%.2f, %.2f, %.2f, %.2f] | fobj = %f\n", k, X(1), X(2), X(3), X(4), fobj(X));
    printf("Iteración %d: Costo mensual = %f\n", k, funcion_costo_climatizacion(X, %F));
end
printf("\n=== SOLUCIÓN FINAL (OPTIMIZADA) ===\n");
printf("Costo mínimo mensual: %f U$D\n", funcion_costo_climatizacion(X, %F));
printf("Horario de Calefacción: Inicia a las %.2fh | Dura %.2fh | Termina a las %.2fh\n", X(1), X(2), X(1)+X(2));
printf("Horario de Refrigeración: Inicia a las %.2fh | Dura %.2fh | Termina a las %.2fh\n", X(3), X(4), X(3)+X(4));


// Presentar nueva solución
printf("\nMinimo aproximado en X = [%f, %f, %f, %f]\n", X(1), X(2), X(3), X(4));
graficar = %T // %T : graficar , %F : NO graficar
funcion_costo_climatizacion(X, graficar);