# dispenser_de_mascotas

Integrantes del grupo:       
Micaela Corvera 
Lucia Gimenez Medina        
Claudia Camila Ramos      

El proyecto consiste en un dispenser para mascotas inteligente, el cual se puede controlar desde un dashboard en adafruit,
y en el cual se puede programar un temporizador o servir comida de forma manual presionando un botón desde un celular o 
cualquier dispositivo con acceso a adafruit. 

Desde el dashboard, también se puede seleccionar el tamaño (aproximado) de la porción.

Los sensores y actuadores utilizados son un servomotor sg90 de giro continuo, y dos sensores de distancia ultrasónicos HCSR04.

En el depósito hay un sensor de distancia, el cual se encarga de informar en el dashboard la cantidad de comida que contiene, y en 
caso de que se encuentre vacío, no servirá comida.

El otro sensor, que se encuentra apuntando al plato de comida, mide la distancia máxima del plato vacío, y la distancia una vez 
que se sirvió la comida. La primera vez que se sirve comida, se toma el tamaño seleccionado desde el dashboard como predeterminado,
y también se toma como distancia de plato lleno, la distancia entre la comida y el sensor después de servir. A partir de ahí, cada vez
que se sirva comida, el dispenser calculará según la distancia, cuanta comida debe servir para llegar a la distancia de plato lleno predeterminada,
y en caso de que ya se encuentre en dicha distancia o la supere, no servirá comida.

El servo funciona con cálculos hechos a partir de la lectura del sensor del plato de comida, estos cálculos determinan por cuanto tiempo deberá girar
el servomotor hasta llegar a la cantidad deseada.

Se puede programar un temporizador, utilizando los teclados numéricos del dashboard. Desde el temporizador se pueden asignar intervalos de tiempo en los que se 
servirá la comida. 
