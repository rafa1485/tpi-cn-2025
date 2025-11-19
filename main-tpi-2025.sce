clear
clc

// Integrantes: Alan Benner; Nicolas Prosman; Mayco Chavez


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
precioEnergiaCalefaccion = 0.045/1000 // [dólares/Wh] - CORREGIDO según PDF

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
    // Potencia de calefacción en función del tiempo
    hr_fin_cal = hr_ini_cal + hr_cal
    if (t/3600) >= hr_ini_cal && (t/3600) <= hr_fin_cal then
        Qc = potenciaCalefaccion;
    else
        Qc = 0;
    end
endfunction

function Qr = Q_refri(t,hr_ini_ref,hr_ref)
    // Potencia de refrigeración en función del tiempo (negativa = extrae calor)
    hr_fin_ref = hr_ini_ref + hr_ref
    if t/3600 >= hr_ini_ref && t/3600 <= hr_fin_ref then
        Qr = -potenciaRefrigeracion;
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

// ============================================================================
// MÉTODO NUMÉRICO: INTEGRACIÓN POR TRAPECIO
// Descripción: Calcula la integral numérica de una función usando la regla
//              del trapecio. Se utiliza para calcular la energía consumida
//              (integral de potencia en el tiempo).
// Fórmula: integral = h/2 * [f(a) + 2*Σf(i) + f(b)]
// ============================================================================
function integral = funcion_integral(t, Q)
    n = length(t);
    h = t(2) - t(1); // Paso de tiempo constante
    
    // Fórmula del Trapecio: integral = h/2 * [f(a) + 2*sum(f(i)) + f(b)]
    integral = Q(1) + Q(n); // Primer y último término
    
    // Sumar todos los términos intermedios (multiplicados por 2)
    for i = 2:(n-1)
        integral = integral + 2*Q(i);
    end
    
    integral = integral * h / 2;
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
        t_actual = t(i);
        T_actual = T(i);
        
        // MÉTODO DE EULER: T(i+1) = T(i) + Dt * f(t(i), T(i))
        dT_dt = f(t_actual, T_actual, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);
        T_nueva = T_actual + Dt * dT_dt;
        
        T = [T, T_nueva];
        t = [t, t_actual + Dt];
    end
    
    
    // CALCULO DEL PERFIL DE CALOR DE CALEFACCION Y REFRIGERACION
    Qc = [Q_calef(0)]
    Qr = [Q_refri(0)]
    
    for i=1:N,
        Qc = [Qc, Q_calef(t(i), hr_ini_cal, hr_cal)];
        Qr_val = Q_refri(t(i), hr_ini_ref, hr_ref);
        Qr = [Qr, abs(Qr_val)]; // Valor absoluto para la integración
    end

    
    // ===== MÉTODO DEL TRAPECIO: Integración numérica =====
    energiaCalefaccionDiaria = funcion_integral(t, Qc) // [Joules]
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
        t_actual = t(i);
        T_actual = T(i);
        
        dT_dt = f(t_actual, T_actual, hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);
        T_nueva = T_actual + Dt * dT_dt;
        
        T = [T, T_nueva];
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

// ============================================================================
// MÉTODO NUMÉRICO: DERIVADAS PARCIALES POR DIFERENCIAS FINITAS CENTRADAS
// Descripción: Calcula las derivadas parciales de la función objetivo usando
//              aproximación numérica con diferencias finitas centradas.
// Fórmula: df/dx ≈ [f(x+h) - f(x-h)] / (2h)
// ============================================================================

function dfx1 = Dfx1(X)
    // Derivada parcial respecto a hr_ini_cal (hora inicio calefacción)
    h = 0.01; // Paso pequeño
    X_mas = X; X_mas(1) = X(1) + h;
    X_menos = X; X_menos(1) = X(1) - h;
    dfx1 = (fobj(X_mas) - fobj(X_menos)) / (2*h);
endfunction

function dfx2 = Dfx2(X)
    // Derivada parcial respecto a hr_cal (duración calefacción)
    h = 0.01;
    X_mas = X; X_mas(2) = X(2) + h;
    X_menos = X; X_menos(2) = X(2) - h;
    dfx2 = (fobj(X_mas) - fobj(X_menos)) / (2*h);
endfunction

function dfx3 = Dfx3(X)
    // Derivada parcial respecto a hr_ini_ref (hora inicio refrigeración)
    h = 0.01;
    X_mas = X; X_mas(3) = X(3) + h;
    X_menos = X; X_menos(3) = X(3) - h;
    dfx3 = (fobj(X_mas) - fobj(X_menos)) / (2*h);
endfunction

function dfx4 = Dfx4(X)
    // Derivada parcial respecto a hr_ref (duración refrigeración)
    h = 0.01;
    X_mas = X; X_mas(4) = X(4) + h;
    X_menos = X; X_menos(4) = X(4) - h;
    dfx4 = (fobj(X_mas) - fobj(X_menos)) / (2*h);
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