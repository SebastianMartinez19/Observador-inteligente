# Observador-inteligente
A lo largo de este proyecto se propone realizar la simulacion de un robot diferencias de dos ruedas actuadas y una rueda loca al cual se le integrara un filtro de kalman alimentado por una red neuronal

# pasos a realizar
1.  Descripcion del modelo fisico.
2.  Simulacion por el metodo de Euler.
3.  Colocacion de filtro de kalman.
4.  Alimentar filtro de kalman

# Paso 1
Del siguinete robot diferencial con dos ruedas actuadas

![image](https://github.com/SebastianMartinez19/Observador-inteligente/assets/106949729/1973a41b-5772-41f2-bb63-1f107548f61c)

Se obtiene las siguientes ecuaciones.

![image](https://github.com/SebastianMartinez19/Observador-inteligente/assets/106949729/b2ebc680-359e-49eb-97dc-5fe715516dc6)

![image](https://github.com/SebastianMartinez19/Observador-inteligente/assets/106949729/1d517503-f08b-4b9a-a47e-15aca83ed9fe)


# Metodo de Euler
Para poder conocer el comportamiento de nuestro robot es necesario integrar los estados, por lo que la manera de hacerlo es dando valores iniciales, en base a esos valores iniciales evaluamos en base a las ecuaciones de estados, el resultado lo multiplicamos por un paso de integracion (entre mas pequeño sea mas preciso es el calculo), actualizamos valores, el resultado es la entrada de la siguinete iteracion.

# Filtro de Kalmnan
El filtro de kalman es un proceso de 4 ecuaciones de las cuales pueden tener diversas aplicaciones, para esta ocasion de utilizara como un observador o estimador, el cual pueda aproximarse a los estados conocidos, es decir, el observador debe estimar el estado de tal manera que el estado estimado y el estado actual obtenido en la simulacion sea 0.

# Alimentacion de filtro de kalman con una red neuronal

El filtro de kalman, como ya se explico anteriormente es un proceso de 4 ecuaciones, de las cuales una en particular se enfoque en actualizar el peso de las ganacias asi como uan respectiva penalizacion, el enfoque de la red neuronal es encontrar esos pesos optimos que provocan que el observador estime los estados con el menor error posible tratando que este sea practicamente 0.

# Todo lo anterior mensionado se encontrara descrito en el documento adjunto a este proyecto asi como el codigo utilizado en matlab, es posible usarlo en python haciendo uso de las librerias matplotlib, para graficar; numpy, para poder realizar las operacion de matrices asi como la funcion tangente hiperbolica la cual es necesaria para la funcion de activacon de nuestra red neuronal.


