# TP5_2024_Carrizo
ADC and DAC

#### i. ¿Cuál es la mínima tensión que pudo observar en la salida del DAC?
Con el buffer habilitado, la tensión en la salida del DAC al cargarle el valor 0 al registro, fue de aproximadamente 70 mV. Con el buffer deshabilitado, dicha tensión bajó hasta los 20 mV.
#### ii. ¿Por qué es diferente a lo esperado?
<p>Según la tabla _6.3.25 DAC electrical characteristics_ de la datasheet de productos _STM32F446xC/E_, el DAC tiene especificaciones de error, offset y no linealidad. Eso quiere decir que el valor medido directamente en el pin del DAC podría no ser exactamente igual al esperado en base al contenido del registro del DAC.<p>
<p>En nuestro caso, con el buffer deshabilitado, notamos que en los valores de prueba, siempre hay +20 mV de lo esperado. Entonces, podríamos inferir que hay un offset de +20 mV.<p>
<p>Con el buffer habilitado, al cargar el valor 0 en el registro, la salida es de 70 mV. Pero, para el resto de los valores testeados, se aprecian un poco más de 20 mV por sobre la tensión deseada (100 mV -> 126 mV, 900 mV -> 935 mV).<p>
<p>La hoja de datos especifica un offset de +- 10 mV para el DAC configurado en 12 bits.<p>
<p>Además del offset, los otros parámetros importantes del DAC son INL (integral non linearity) and DNL (differential non linearity). En nuestro caso los valores máximos son de +- 4 LSB (+- 3,22 mV) y +- 2 LSB (1,61 mV) respectivamente.<p>

#### iii. ¿Qué se puede hacer para mejorar la linealidad?
Para mejorar la linealidad se podría hacer una calibración del DAC por firmware. Es decir, conectar la salida del DAC a una entrada ADC, medir la tensión para diferentes valores de registro de DAC, y calcular el error. Posteriormente dicho error será considerado a la hora de escribir el registro del DAC.
