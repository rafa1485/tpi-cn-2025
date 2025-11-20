clear
clc


// DEFINICION PARA SALIDA GRAFICA
function grafico_salida(t,T,Qc,Qr,costoRefrigeracion,costoCalefaccion,titulo)
    f = figure()
    f.figure_name = titulo
    
    subplot(4,1,1)
    plot(t/3600,T)
    xlabel('Tiempo [horas]')
    ylabel('Temperatura [°C]')
    title('Evolución de la Temperatura Interior')
    xgrid()
    
    subplot(4,1,2)
    plot(t/3600,Qc,'r')
    xlabel('Tiempo [horas]')
    ylabel('Potencia [W]')
    title('Perfil de Calefacción')
    xgrid()
    
    subplot(4,1,3)
    plot(t/3600,Qr,'b')
    xlabel('Tiempo [horas]')
    ylabel('Potencia [W]')
    title('Perfil de Refrigeración')
    xgrid()
    
    subplot(4,1,4)
    xstring(0.1,0.33,"Costo Refrigeración= U$D "+string(costoRefrigeracion),0,0)
    xstring(0.1,0.66,"Costo Calefacción= U$D "+string(costoCalefaccion),0,0)
    xstring(0.1,0.1,"Costo Total= U$D "+string(costoCalefaccion+costoRefrigeracion),0,0)
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
    Qp = conductanciaPiso * (15 - T_int); // Aca se considera que la Temperatura del suelo es constante e igual a 15 °C
endfunction

function Qe = Q_edif(t, T_int)
    T_ext = T_exterior(t) 
    Re = 1/conductanciaEdificacion; // Resistencia a la transferencia de calor por la pared de la edificación
    Rc = 1/conductanciaConveccionEdificacion; // Resistencia a la transferencia de calor por convección en la edificación
    conductanciaTotalEdificacion = 1/(Re + Rc);
    Qe = conductanciaTotalEdificacion * (T_ext - T_int)
endfunction

function Qc = Q_calef(t, T_int, hr_ini_cal, hr_cal)
    hr_fin_cal = hr_ini_cal + hr_cal
    T_setpoint = 20;  // Temperatura objetivo [°C]
    T_histeresis = 2; // Banda de histéresis [°C]
    
    // Solo operar en el horario permitido
    if (t/3600) >= hr_ini_cal && (t/3600) <= hr_fin_cal then
        // Control por temperatura: encender si T < 20-2 = 18°C
        if T_int < (T_setpoint - T_histeresis) then
            Qc = potenciaCalefaccion;
        else
            Qc = 0;
        end
    else
        Qc = 0;
    end
endfunction

function Qr = Q_refri(t, T_int, hr_ini_ref, hr_ref)
    hr_fin_ref = hr_ini_ref + hr_ref
    T_setpoint = 20;  // Temperatura objetivo [°C]
    T_histeresis = 2; // Banda de histéresis [°C]
    
    // Solo operar en el horario permitido
    if (t/3600) >= hr_ini_ref && (t/3600) <= hr_fin_ref then
        // Control por temperatura: encender si T > 20+2 = 22°C
        if T_int > (T_setpoint + T_histeresis) then
            Qr = 0;
        else
            Qr = potenciaRefrigeracion;  // Negativo porque EXTRAE calor del edificio
        end
    else
        Qr = 0;
    end
endfunction


function Qt = Q_total(t, T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    Qp = Q_piso(T_int);
    Qe = Q_edif(t,T_int);
    Qc = Q_calef(t, T_int, hr_ini_cal, hr_cal)  // Ahora recibe T_int
    Qr = Q_refri(t, T_int, hr_ini_ref, hr_ref)  // Ahora recibe T_int
    Qt = Qp + Qe + Qc + Qr;
endfunction

function dT = f(t,T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref)
    dT = Q_total(t,T_int, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref) / capacidadCalorificaEdificio;
endfunction


function costoClimatizacion = funcion_costo_climatizacion(X, graficar, titulo)

    
    // DEFINICION VARIABLES DE CONTROL TEMPERATURA
    hr_ini_cal = X(1)
    hr_cal = X(2)
    hr_ini_ref = X(3)
    hr_ref = X(4)
    
    
    // PROGRAMACION METOD0 DE EULER PARA CALCULAR LA EVOLUCION DE LA TEMPERATURA
    T_ini = 19.5; // El TP pide que la temperatura inicial sea de 20 +- 2 °C
    T = [T_ini]
    T_ext = [T_exterior(0)]
    Dt = 36;
    t = [0]
    N = (24 * 3600)/ Dt;
    
    
    for i=1:N,
        // COMPLETAR METOD0 DE EULER
        
        T_actual = T(i);       // 1. Obtengo la temperatura actual del vector
        t_actual = t(i);       // 2. Obtengo el tiempo actual
        
        // 3. Calculo la derivada (dT/dt) usando la función física 'f'
        // Esto me dice qué tan rápido cambia la temperatura en este instante
        dT = f(t_actual, T_actual, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);
        
        // 4. Aplico EULER: Temp Futura = Temp Actual + (Cambio * Paso de tiempo)
        T_siguiente = T_actual + dT * Dt;
        
        // 5. Guardo los nuevos valores en los vectores (agregando al final de la lista)
        T = [T, T_siguiente];
        t = [t, t_actual + Dt];

        // Al finalizar el METOD0 de Euler se debe tener un Vector FILA 'T'
        // con las temperaturas para cada tiempo en SEGUNDOS que se guarda en 
        // el vector 't'
    end
    
    
    // CALCULO DEL PERFIL DE CALOR DE CALEFACCION Y REFRIGERACION
    // Para gráficos y costos usamos valor absoluto (potencia consumida)
    Qc = [Q_calef(0, T(1), hr_ini_cal, hr_cal)]
    Qr = [abs(Q_refri(0, T(1), hr_ini_ref, hr_ref))]  // Valor absoluto para graficar
    
    for i=1:N,
        Qc = [Qc, Q_calef(t(i), T(i), hr_ini_cal, hr_cal)];
        Qr = [Qr, abs(Q_refri(t(i), T(i), hr_ini_ref, hr_ref))]  // Valor absoluto para graficar
    end

    
    // INTEGRACION DE LA ENERGIA DE CALEFACCION A LO LARGO DEL DIA (JOULES)
    // Programar una funcion_integral(t,Qc), que calcule la Energía total 
    // de Calefacción mediente la integral de Qc en funcion de t // [Joules]
    // Método del Trapecio: Area = (base * (altura1 + altura2)) / 2
    function energia = funcion_integral_calefaccion(t,Qc)
        energia = 0
        for i=1:(length(t)-1),
            Dt = t(i+1) - t(i)
            energia = energia + (Qc(i) + Qc(i+1)) * Dt / 2
        end
    endfunction
    
    energiaCalefaccionDiaria = funcion_integral_calefaccion(t, Qc)
    
    
    // INTEGRACION DE LA ENERGIA DE REFRIGERACION A LO LARGO DEL DIA (JOULES)
    // Programar una funcion_integral(t,Qr), que calcule la Energía total 
    // de Refrigeración mediente la integral de Qr en funcion de t // [Joules]
    // Método del Trapecio: Area = (base * (altura1 + altura2)) / 2
    function energia = funcion_integral_refrigeracion(t,Qr)
        energia = 0
        for i=1:(length(t)-1),
            Dt = t(i+1) - t(i)
            energia = energia + (Qr(i) + Qr(i+1)) * Dt / 2
        end
    endfunction
    
    energiaRefrigeracionDiaria = funcion_integral_refrigeracion(t, Qr) // [Joules]

    energiaCalefaccionMensual_Wh = energiaCalefaccionDiaria * 30 / 3600
    
    costoCalefaccion = precioEnergiaCalefaccion * energiaCalefaccionMensual_Wh
    
    energiaRefrigeracionMensual_Wh = energiaRefrigeracionDiaria * 30 / 3600
    
    costoRefrigeracion = precioEnergiaRefrigeracion * energiaRefrigeracionMensual_Wh
    
    costoClimatizacion = costoCalefaccion + costoRefrigeracion

    if graficar then
        printf("\n--- RESUMEN DE COSTOS ---\n")
        printf("Costo Calefacción: U$D %.2f\n", costoCalefaccion)
        printf("Costo Refrigeración: U$D %.2f\n", costoRefrigeracion)
        printf("Costo Total: U$D %.2f\n\n", costoClimatizacion)
        grafico_salida(t,T,Qc,Qr,costoRefrigeracion,costoCalefaccion, titulo)
    end
endfunction


function temperatura = funcion_perfil_temperatura(X)
    // DEFINICION VARIABLES DE CONTROL TEMPERATURA
    hr_ini_cal = X(1) // "Hora a la que se enciende la calefacción"
    hr_cal = X(2) // "Hora a la que se apaga la calefacción"
    hr_ini_ref = X(3) // "Hora a la que se enciende la refrigeración"
    hr_ref = X(4) // "Hora a la que se apaga la refrigeración"
    
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
        T_actual = T(i);       // 1. Obtengo la temperatura actual del vector
        t_actual = t(i);       // 2. Obtengo el tiempo actual
        
        // 3. Calculo la derivada (dT/dt)
        // Esto me dice qué tan rápido cambia la temperatura en este instante
        dT = f(t_actual, T_actual, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);
        
        // 4. Aplico EULER: Temp Futura = Temp Actual + (Cambio * Paso de tiempo)
        T_siguiente = T_actual + dT * Dt;
        
        // 5. Guardo los nuevos valores en los vectores (agregando al final de la lista)
        T = [T, T_siguiente];
        t = [t, t_actual + Dt];
    
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
     
printf("\n=== SOLUCION INICIAL ===\n")
graficar = %T // %T : graficar , %F : NO graficar
funcion_costo_climatizacion(X, graficar, "Solución Inicial");

function fcc = fobj(X)
    epsilon1 = 10
    epsilon2 = 100
    epsilon3 = 0.1
    epsilon4 = 0.1
    temperatura_diaria = funcion_perfil_temperatura(X)
    diferencia_cuad_inicio_fin = (temperatura_diaria($) - temperatura_diaria(1))^2
    varianza_temperatura = stdev(temperatura_diaria)^2
    fcc = funcion_costo_climatizacion(X, %F, "Optimización FCC") + epsilon1 * diferencia_cuad_inicio_fin + epsilon2 * varianza_temperatura + epsilon3*1/X(2) + epsilon4*1/X(4)
endfunction

// DEFINICION DE DERIVADAS PARCIALES NUMERICAS

function dfx1 = Dfx1(X)
    // Calcular derivada de fobj() en respecto de x1
    h = 0.01;
    X1_plus = X;
    X1_plus(1) = X1_plus(1) + h;
    X1_minus = X;
    X1_minus(1) = X1_minus(1) - h;
    dfx1 = (fobj(X1_plus) - fobj(X1_minus)) / (2 * h);
endfunction

function dfx2 = Dfx2(X)
    // Calcular derivada de fobj() en respecto de x2
    h = 0.01;
    X2_plus = X;
    X2_plus(2) = X2_plus(2) + h;
    X2_minus = X;
    X2_minus(2) = X2_minus(2) - h;
    dfx2 = (fobj(X2_plus) - fobj(X2_minus)) / (2 * h);
endfunction

function dfx3 = Dfx3(X)
    // Calcular derivada de fobj() en respecto de x3
    h = 0.01;
    X3_plus = X;
    X3_plus(3) = X3_plus(3) + h;
    X3_minus = X;
    X3_minus(3) = X3_minus(3) - h;
    dfx3 = (fobj(X3_plus) - fobj(X3_minus)) / (2 * h);
endfunction

function dfx4 = Dfx4(X)
    // Calcular derivada de fobj() en respecto de x4 
    h = 0.01; // Paso de variación pequeño
    X4_plus = X; // Inicialización de X4_plus
    X4_plus(4) = X4_plus(4) + h;
    X4_minus = X; // Inicialización de X4_minus
    X4_minus(4) = X4_minus(4) - h; // Corregido el orden de las operaciones
    dfx4 = (fobj(X4_plus) - fobj(X4_minus)) / (2 * h); // Derivada numérica por diferencia central
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
alpha = 0.01  // Tasa de aprendizaje (paso más agresivo para converger más rápido)
max_iter = 100  // Número máximo de iteraciones
tol = 0.01      // Tolerancia de convergencia

for k = 1:max_iter // Inicia bucle que itera hasta max_iter veces
    printf("Iteracion %d/%d... ", k, max_iter)
    // COMPLETAR EL METOD0 del GRADIENTE DESCENDENTE para minimizar 'fobj'   
    g = grad_f(X); // Calcula el gradiente (vector con 4 derivadas parciales) en el punto actual X
    X_new = X - alpha * g; // Calcula nueva posición: se mueve en dirección opuesta al gradiente con paso alpha
    norma = norm(X_new - X);
    printf("norma=%.4f\n", norma)
    if norma < tol then // Verifica si la distancia euclidiana entre X_new y X es menor que la tolerancia
        printf("\n*** CONVERGENCIA ALCANZADA en iteracion %d ***\n", k)
        break; // Sale del bucle anticipadamente si convergió (cambio muy pequeño)
    end
    X = X_new; // Actualiza X con el nuevo valor para la próxima iteración
end

if k == max_iter then
    printf("\n*** Se alcanzo el maximo de iteraciones sin convergencia total ***\n")
end

// Presentar nueva solución
printf("\n\n=== SOLUCION OPTIMIZADA ===\n")
printf("Inicio calefacción: %.2f horas\n", X(1))
printf("Duracion calefacción: %.2f horas\n", X(2))
printf("Inicio refrigeración: %.2f horas\n", X(3))
printf("Duracion refrigeración: %.2f horas\n", X(4))
printf("\nVector X optimizado = [%.4f, %.4f, %.4f, %.4f]\n", X(1), X(2), X(3), X(4));

graficar = %T // %T : graficar , %F : NO graficar
funcion_costo_climatizacion(X, graficar, "Solución Optimizada");
