clear
clc


// DEFINICION PARA SALIDA GRAFICA (MODIFICADA CON ETIQUETAS DE EJES)
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
    plot(Qc,'r')
    title("Potencia de Calefacción (Qc)")
    xlabel("Pasos de Tiempo") // Mantener en Pasos de Tiempo para Q, o usar t/3600
    ylabel("Potencia (W)")
    
    // 3. POTENCIA DE REFRIGERACIÓN
    subplot(4,1,3)
    plot(Qr,'b')
    title("Potencia de Refrigeración (Qr)")
    xlabel("Pasos de Tiempo")
    ylabel("Potencia (W)")
    
    // 4. COSTOS MENSUALES
    subplot(4,1,4)
    // Mostrando el Costo Total en una posición adicional
    costoTotal = costoRefrigeracion + costoCalefaccion;
    xstring(0.1,0.1,"Costo Total Climatización= U$D"+string(costoTotal),0,0) // Costo Total
    xstring(0.1,0.33,"Costo Refrigeración= U$D"+string(costoRefrigeracion),0,0)
    xstring(0.1,0.66,"Costo Calefacción= U$D"+string(costoCalefaccion),0,0)
    xlabel("Costos Mensuales")
    ylabel(" ") // Dejar el eje Y vacío o con un comentario si se desea
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
precioEnergiaCalefaccion = 0.0000139 // [dólares/Wh]

// CALCULO DEL COSTO DE LA ENERGIA DE CALEFACCION
//poderCalorificoGas = 10.8 //[kWh/m3]
//precioM3Gas = 180 // [$/m3]
//precio_energia_Gas_Pesos_kWh = precioM3Gas / poderCalorificoGas / 0.8 // Considerando un rendimiento termico de 0.8
//precioDolar_Pesos = 1500 
//precio_energia_Gas_USD_kWh = precio_energia_Gas_Pesos_kWh / precioDolar_Pesos
//precio_energia_Gas_USD_Wh = precio_energia_Gas_USD_kWh / 1000

potenciaRefrigeracionUnitaria = 3 // Potencia de refrigeración por metro cuadrado de superficie construida [W/m2]
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
    Re = 1/conductanciaEdificacion;
// Resistencia a la transferencia de calor por la pared de la edificación
    Rc = 1/conductanciaConveccionEdificacion;
// Resistencia a la transferencia de calor por convección en la edificación
    conductanciaTotalEdificacion = 1/(Re + Rc);
    Qe = conductanciaTotalEdificacion * (T_ext - T_int)
endfunction

function Qc = Q_calef(t,hr_ini_cal,hr_cal)
    hr_fin_cal = hr_ini_cal + hr_cal
    if (t/3600) >= hr_fin_cal && (t/3600) <= hr_ini_cal then
        Qc = 0;
    else
        Qc = potenciaCalefaccion;
    end
endfunction

function Qr = Q_refri(t,hr_ini_ref,hr_ref)
    hr_fin_ref = hr_ini_ref + hr_ref
    if t <= hr_ini_ref*3600 then
        Qr = 0;
    elseif t <= hr_fin_ref*3600 then
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
    Dt = t(2) - t(1); // Paso de tiempo
    N = length(Q);
    integral = 0;
    
    // Suma de la Regla del Trapecio: Sum((Q_i + Q_{i-1})/2 * Dt)
    for i = 2:N
        integral = integral + ((Q(i) + Q(i-1)) / 2) * Dt;
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
    // IMPLEMENTACION DEL METODO DE EULER
        t_nuevo = t(i) + Dt;
        t = [t, t_nuevo];
        T_i = T(i) + Dt * f(t(i), T(i), hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);
        T = [T, T_i];
    
    end
    
    
    // CALCULO DEL PERFIL DE CALOR DE CALEFACCION Y REFRIGERACION
    Qc = [Q_calef(0, hr_ini_cal, hr_cal)]
    Qr = [Q_refri(0, hr_ini_ref, hr_ref)]
    
    for i=1:N,
        Qc = [Qc, Q_calef(t(i), hr_ini_cal, hr_cal)];
        Qr = [Qr, Q_refri(t(i), hr_ini_ref, hr_ref)]
    end

    
    // INTEGRACION DE LA ENERGIA DE CALEFACCION A LO LARGO DEL DIA (JOULES)
    energiaCalefaccionDiaria = funcion_integral(t, Qc) // [Joules]
    
    
    // INTEGRACION DE LA ENERGIA DE REFRIGERACION A LO LARGO DEL DIA (JOULES)
    energiaRefrigeracionDiaria = funcion_integral(t, Qr) // [Joules]
   
    
    energiaCalefaccionMensual_Wh = energiaCalefaccionDiaria * 30 / 3600
    
    costoCalefaccion = precioEnergiaCalefaccion * energiaCalefaccionMensual_Wh
    
    // Corregida la conversión de unidades (multiplicación por 30 días y división por 3600 J/Wh)
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
    // IMPLEMENTACION DEL METODO DE EULER
        t_nuevo = t(i) + Dt;
        t = [t, t_nuevo];
        T_i = T(i) + Dt * f(t(i), T(i), hr_ini_cal, hr_cal, hr_ini_ref, hr_ref);
        T = [T, T_i];
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
    h = 1e-4 // Paso de diferencia finita
    Xh = X;
    Xh(1) = X(1) + h;
    dfx1 = (fobj(Xh) - fobj(X)) / h;
endfunction

function dfx2 = Dfx2(X)
    h = 1e-4 // Paso de diferencia finita
    Xh = X;
    Xh(2) = X(2) + h;
    dfx2 = (fobj(Xh) - fobj(X)) / h;
endfunction

function dfx3 = Dfx3(X)
    h = 1e-4 // Paso de diferencia finita
    Xh = X;
    Xh(3) = X(3) + h;
    dfx3 = (fobj(Xh) - fobj(X)) / h;
endfunction

function dfx4 = Dfx4(X)
    h = 1e-4 // Paso de diferencia finita
    Xh = X;
    Xh(4) = X(4) + h;
    dfx4 = (fobj(Xh) - fobj(X)) / h;
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
alpha = 0.01
max_iter = 100
tol = 0.01

for k = 1:max_iter
    // IMPLEMENTACION DEL METODO del GRADIENTE DESCENDENTE
    gradiente = grad_f(X);
    X_nuevo = X - alpha * gradiente;
    
    // Verificar si la magnitud del gradiente es menor que la tolerancia
    if norm(gradiente) < tol then
        printf("\nConvergencia alcanzada en la iteración: %d\n", k);
        break
    end
    
    X = X_nuevo;
end

// Presentar nueva solución


printf("\nMinimo aproximado en X = [%f, %f, %f, %f]\n", X(1), X(2), X(3), X(4));
graficar = %T // %T : graficar , %F : NO graficar
funcion_costo_climatizacion(X, graficar);