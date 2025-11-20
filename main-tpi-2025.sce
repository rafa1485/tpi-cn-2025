clear
clc


// DEFINICION PARA SALIDA GRAFICA
function grafico_salida(t,T,Qc,Qr,costoRefrigeracion,costoCalefaccion)
    figure()
    subplot(4,1,1)
    plot(t/3600,T)
    
    subplot(4,1,2)
    plot(Qc,'r')
    
    subplot(4,1,3)
    plot(Qr,'b')
    
    subplot(4,1,4)
    xstring(0.1,0.33,"Costo Refrigeración= U$D"+string(costoRefrigeracion),0,0)
    xstring(0.1,0.66,"Costo Calefacción= U$D"+string(costoCalefaccion),0,0)
endfunction

// =================================================================
// 1. DEFINICIÓN DE PARÁMETROS AMBIENTALES Y GEOMÉTRICOS DEL EDIFICIO
// =================================================================

// Parámetros de la Curva de Temperatura Exterior (Perfil de 24 horas)
TAmbMax = 24 //"Máxima Temperatura Ambiente"
TAmbMin = 10 //"Mínima Temperatura Ambiente"
InicioSubida = 6 //"Hora en la que empieza a incrementar la temperatura (h)"
FinSubida = 11 //"Hora en la que la temperatura alcanza el máximo (h)"
InicioBajada = 14 //"Hora en la que empieza a decrementar la temperatura (h)"
FinBajada = 19 //"Hora en la que la temperatura alcanza el mínimo de nuevo (h)"


// Parámetros Geométricos
superficieEdificacion=100 // [m2] (Paredes y Techo)
superficiePiso=70 // [m2] (Área en contacto con el suelo)


// Parámetros Térmicos de la Edificación (Paredes y Techo)
espesorEdificacion = 0.3 // [m]
coeficienteConductanciaEdificacion = 0.4 / espesorEdificacion // [W/K/m2] (U-value sin convección)
conductanciaEdificacion = coeficienteConductanciaEdificacion * superficieEdificacion // [W/K]


// Parámetros Térmicos del Piso
espesorAislacionPiso = 0.05 // [m]
coeficienteConductanciaPiso = 0.02 / espesorAislacionPiso // [W/K/m2] (U-value del piso)
conductanciaPiso = superficiePiso*coeficienteConductanciaPiso // [W/K]


// Parámetros de Calefacción
potenciaCalefaccionUnitaria = 10 // Potencia de calefacción por metro cuadrado de superficie construida [W/m2]
potenciaCalefaccion = potenciaCalefaccionUnitaria * superficiePiso // Potencia total de calefacción [W]
precioEnergiaCalefaccion = 0.0000139 // Precio de la energía de calefacción [dólares/Wh]

// Código comentado para un cálculo alternativo de costo de gas (no usado)
// CALCULO DEL COSTO DE LA ENERGIA DE CALEFACCION
//poderCalorificoGas = 10.8 //[kWh/m3]
//precioM3Gas = 180 // [$/m3]
//precio_energia_Gas_Pesos_kWh = precioM3Gas / poderCalorificoGas / 0.8 // Considerando un rendimiento termico de 0.8
//precioDolar_Pesos = 1500 
//precio_energia_Gas_USD_kWh = precio_energia_Gas_Pesos_kWh / precioDolar_Pesos
//precio_energia_Gas_USD_Wh = precio_energia_Gas_USD_kWh / 1000

// Parámetros de Refrigeración
potenciaRefrigeracionUnitaria = 3 // Potencia de refrigeración por metro cuadrado de superficie construida [W/m2] (NOTA: Revisar valor en PDF)
potenciaRefrigeracion = potenciaRefrigeracionUnitaria * superficiePiso // Potencia total de refrigeración [W]
precioEnergiaRefrigeracion = 0.12/1000 // Precio de la energía de refrigeración [dólares/Wh]


// Parámetros de Capacidad Térmica del Edificio (Inercia)
masaUnitaria = 150 // Masa de edificio por unidad de superficie de construcción [kg/m2]
capacidadCalorificaEspecifica = 800 // Capacidad Calorífica por kg del material de construcción [J/kg/K]
capacidadCalorificaUnitaria = masaUnitaria * capacidadCalorificaEspecifica // [J/K/m2]
capacidadCalorificaEdificio = capacidadCalorificaUnitaria * superficiePiso // Capacidad total del aire del edificio [J/K]


// Parámetros de Convección (Transferencia de Calor Aire-Superficie Exterior)
h = 18 // Coeficiente de transferencia de calor por convección [W/m2/K]
conductanciaConveccionEdificacion = h * superficieEdificacion;

// =================================================================
// 2. FUNCIONES DEL MODELO TERMODINÁMICO
// =================================================================

// Función que calcula la Temperatura Exterior (T_ext) en un tiempo 't' (en segundos).
// Sigue un perfil trapezoidal basado en las horas definidas.
function T_ext = T_exterior(t)
    // Conversión de tiempo a horas (para comparar con las variables Inicio/Fin)
    if t <= InicioSubida*3600 then
        T_ext = TAmbMin; // Antes de la subida: Temperatura Mínima
    elseif t <= FinSubida*3600 then
        // Tramo de subida (Lineal)
        T_ext = ((TAmbMax - TAmbMin)/(FinSubida - InicioSubida))*(t/3600 - InicioSubida) + TAmbMin;
    elseif t <= InicioBajada*3600 then
        T_ext = TAmbMax; // Pico de temperatura: Temperatura Máxima
    elseif t <= FinBajada*3600 then
        // Tramo de bajada (Lineal)
        T_ext = ((TAmbMin - TAmbMax)/(FinBajada - InicioBajada))*(t/3600 - InicioBajada) + TAmbMax;
    else
        T_ext = TAmbMin; // Después de la bajada: Temperatura Mínima
    end
endfunction

// Flujo de Calor a través del Piso (Qp).
// Se asume T_suelo = 15°C (constante). Qp es positivo si T_suelo > T_int.
function Qp = Q_piso(T_int)
    Qp = conductanciaPiso * (15 - T_int);
endfunction

// Flujo de Calor a través de la Edificación (Paredes/Techo) (Qe).
// Considera la conductancia del material y la convección del aire exterior.
function Qe = Q_edif(t, T_int)
    T_ext = T_exterior(t) // Temperatura exterior en el tiempo t
    
    // Resistencias Térmicas
    Re = 1/conductanciaEdificacion; // Resistencia a la transferencia de calor por la pared de la edificación
    Rc = 1/conductanciaConveccionEdificacion; // Resistencia a la transferencia de calor por convección en la edificación
    
    // Conductancia Total Edificación (Serie de Resistencias)
    conductanciaTotalEdificacion = 1/(Re + Rc);
    
    // Flujo de calor: Conductancia Total * Diferencia de Temperatura
    Qe = conductanciaTotalEdificacion * (T_ext - T_int)
endfunction

// Flujo de Calor aportado por la Calefacción (Qc).
// Qc es constante si el sistema está encendido, cero si está apagado.
function Qc = Q_calef(t,hr_ini_cal,hr_cal)
    hr_fin_cal = hr_ini_cal + hr_cal // Hora de fin de la calefacción
    
    // Condición de encendido/apagado.
    // NOTA: La lógica de la condición de rango (>= hr_fin_cal && <= hr_ini_cal) parece ser incorrecta para un rango continuo.
    if (t/3600) >= hr_fin_cal && (t/3600) <= hr_ini_cal then
        Qc = 0; // Apagado
    else
        Qc = potenciaCalefaccion; // Encendido (Flujo positivo)
    end
endfunction

// Flujo de Calor extraído por la Refrigeración (Qr).
// Qr es constante si el sistema está encendido, cero si está apagado.
function Qr = Q_refri(t,hr_ini_ref,hr_ref)
    hr_fin_ref = hr_ini_ref + hr_ref // Hora de fin de la refrigeración
    
    // Condición de encendido/apagado (por tramos).
    if t <= hr_ini_ref*3600 then
        Qr = 0;
    elseif t <= hr_fin_ref*3600 then
        Qr = potenciaRefrigeracion; // Encendido (Flujo positivo) - NOTA: Este flujo debería ser negativo en Q_total.
    else
        Qr = 0;
    end
endfunction


// Flujo de Calor Total (Qt) sobre el volumen de control del aire interior.
// Qt = Qp (Piso) + Qe (Edificación) + Qc (Calefacción) + Qr (Refrigeración)
// NOTA: Para que Qr sea una extracción, debería ser negativo aquí o en Q_refri.
function Qt = Q_total(t, T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    Qp = Q_piso(T_int);
    Qe = Q_edif(t,T_int);
    Qc = Q_calef(t,hr_ini_cal,hr_cal)
    Qr = Q_refri(t,hr_ini_ref,hr_ref)
    Qt = Qp + Qe + Qc + Qr;
endfunction

// Función de la Derivada (dT/dt = f(t, T_int))
// Es la base del método de Euler. Sigue la Ley de Newton de Enfriamiento / Conservación de Energía.
// dT/dt = Qt / CapacidadCalorifica
function dT = f(t,T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    dT = Q_total(t,T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref) / capacidadCalorificaEdificio;
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
    // COMPLETAR METOD0 DE EULER
    // Al finalizar el METOD0 de Euler se debe tener un Vector FILA 'T'
    // con las temperaturas para cada tiempo en SEGUNDOS que se guarda en 
    // el vector 't'
    tc = t(i);                                // tiempo actual (s)
    k = f(tc, T(i), hr_ini_cal, hr_cal, hr_ini_ref, hr_ref); // derivada dT/dt
    t = [t, tc+Dt];                           // nuevo tiempo
    T = [T, T(i) + Dt * k];  
    
    end
    
    
    // CALCULO DEL PERFIL DE CALOR DE CALEFACCION Y REFRIGERACION
    Qc = [Q_calef(0)]
    Qr = [Q_refri(0)]
    
    for i=1:N,
        Qc = [Qc, Q_calef(t(i), hr_ini_cal, hr_cal)];
        Qr = [Qr, Q_refri(t(i), hr_ini_ref, hr_ref)]
    end

    
    // INTEGRACION DE LA ENERGIA DE CALEFACCION A LO LARGO DEL DIA (JOULES)
    energiaCalefaccionDiaria = 0
    // Programar una funcion_integral(t,Qc), que calcule la Energía total 
    // de Calefacción mediente la integral de Qc en funcion de t // [Joules]
    
    
    // INTEGRACION DE LA ENERGIA DE REFRIGERACION A LO LARGO DEL DIA (JOULES)
    energiaRefrigeracionDiaria = 0 // [Joules]
    // Programar una funcion_integral(t,Qr), que calcule la Energía total 
    // de Refrigeración mediente la integral de Qr en funcion de t // [Joules]
    
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
    // COMPLETAR METOD0 DE EULER
    // Al finalizar el METOD0 de Euler se debe tener un Vector FILA 'T'
    // con las temperaturas para cada tiempo en SEGUNDOS que se guarda en 
    // el vector 't'
    
    end
    
    temperatura = T
endfunction

// PROGAMACION OPTIMIZACIÓN CON GRADIETNE DESCENDENTE
inicioCalefaccion = 0 // "Hora a la que se enciende la Refrigeracion"
finCalefaccion = 24 // "Hora a la que se apaga la refrigeración"
inicioRefrigeracion = 8 // "Hora a la que se enciende la Refrigeracion"
finRefrigeracion = 6 // "Hora a la que se apaga la refrigeración"

X = [inicioCalefaccion;
     finCalefaccion;
     inicioRefrigeracion;
     finRefrigeracion]
     
graficar = %T // %T : graficar , %F : NO graficar
funcion_costo_climatizacion(X, graficar);

function fcc = fobj(X)
    epsilon1 = 10
    epsilon2 = 100
    epsilon3 = 0.1
    epsilon4 = 0.1
    temperatura_diaria = funcion_perfil_temperatura(X)
    diferencia_cuad_inicio_fin = (temperatura_diaria($) - temperatura_diaria(1))^2
    varianza_temperatura = stdev(temperatura_diaria)^2
    fcc = funcion_costo_climatizacion(X, %F) + epsilon1 * diferencia_cuad_inicio_fin + epsilon2 * varianza_temperatura + epsilon3*1/X(2) + epsilon4*1/X(4)
endfunction

// DEFINICION DE DERIVADAS PARCIALES NUMERICAS

function dfx1 = Dfx1(X)
    // Calcular derivada de fobj() en respecto de x1
endfunction

function dfx2 = Dfx2(X)
    // Calcular derivada de fobj() en respecto de x2 
endfunction

function dfx3 = Dfx3(X)
    // Calcular derivada de fobj() en respecto de x3 
endfunction

function dfx4 = Dfx4(X)
    // Calcular derivada de fobj() en respecto de x4 
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
tol = 0.01

for k = 1:max_iter
    // COMPLETAR EL METOD0 del GRADIENTE DESCENDENTE para minimizar 'fobj'    
end

// Presentar nueva solución


printf("\nMinimo aproximado en X = [%f, %f, %f, %f]\n", X(1), X(2), X(3), X(4));
graficar = %T // %T : graficar , %F : NO graficar
funcion_costo_climatizacion(X, graficar);
