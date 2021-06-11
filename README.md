# Follower
ROS package for transmission line tracking using YOLO, MAVROS and PX4 Autopilot.

![alt text](https://github.com/lcfdiniz/Follower/blob/master/follower_img.png?raw=true)

## Guia do usuário do pacote “follower”

O pacote “follower” é um pacote ROS criado para permitir o controle Offboard de um VANT simulado. Este pacote possui scripts escritos em linguagem python utilizados para a comunicação com um sistema de piloto automático (Autopilot) por mensagens MAVLink através do pacote MAVROS. 

### Funcionamento básico

O funcionamento básico do pacote é feito da seguinte maneira: dois scripts, detection_client.py e detection_server.py, são inicializados. Assim como os nomes sugerem, estes scripts podem se comunicar pelo modelo cliente-servidor. 

O nó cliente é aquele que de fato envia mensagens contendo informações de setpoint para o Autopilot. Para isso, este anuncia no tópico /mavros/setpoint_raw/local, onde estas mensagens serão publicadas. Além de se atualizar com as mensagens de realimentação dos sensores providas pelo sistema controlador de voo,  o nó cliente recebe mensagens do tópico /follower/commands. Neste tópico são publicadas mensagens enviadas diretamente pelo usuário, permitindo a modificação em certos aspectos do programa. Examinando o tópico:

### /follower/commands

Este tópico recebe mensagens do tipo Follower.msg. Estas mensagens customizadas possuem os seguintes campos:
    
    • x: variável do tipo float64
    
    • y: variável do tipo float64
    
    • z: variável do tipo float64
    
    • yaw: variável do tipo float64
    
    • mode: variável do tipo uint8

Uma mensagem pode ser publicada diretamente pelo terminal Ubuntu através do comando rostopic pub. A sintaxe completa pode ser vista abaixo:
    
    • rostopic pub -1 follower/commands follower/Follower -- x y z yaw mode

onde -1 indica que a mensagem deve ser publicada apenas uma vez. “follower/commands” é o tópico de publicação. “follower/Follower” é o tipo da mensagem. Os dois traços indicam ao analisador que não serão enviadas opções de linha de comando adicionais. Por fim, os campos da mensagem devem ser preenchidos de acordo com o tipo de variável descrito anteriormente.

Primeiramente, os modos disponíveis serão descritos:
    
    • Modo “0”: Takeoff. Este modo é utilizado para realizar a decolagem do VANT para a altura (campo z) especificada;
    
    • Modo “1”: Set Position. Este modo levará o VANT para as posições globais (x,y,z,yaw) especificadas em seus respectivos campos;
    
    • Modo “2”: Set Relative Position. De maneira similar ao modo anterior, este levara o VANT para a posição global que é a soma entre a posição atual do mesmo e os valores especificados nos campos (x,y,z,yaw);
    
    • Modo “3”: Hold. Este modo mantém o VANT na posição global atual;
    
    • Modo “4”: Get Position. Este modo imprime na tela a posição global atual do VANT;
    
    • Modo “5”: Follow Lines. Este modo implementa o seguidor de linha de transmissão, e será explicado em maiores detalhes mais a frente;
    
    • Modo “6”: Land In Line. Este modo permite o pouso sobre a linha de transmissão, e será explicado em maiores detalhes mais a frente;
    
    • Modo “7”: Return To Land. Retorna o VANT à sua posição de origem.

A tabela abaixo apresenta a relação de cada modo com os demais campos da mensagem:

| mode | x | y | z | yaw |
|:-:|:-:|:-:|:-:|:-:|
| 0 | - | - | Altitude (m) | - |
| 1 | Posição x (m) | Posição y (m) | Altitude (m) | Guinada (rad) |
| 2 | Delta x (m) | Delta y (m) | Delta z (m) | Delta yaw (rad) |
| 3 | - | - | - | - |
| 4 | - | - | - | - |
| 5 | - | - | - | - |
| 6 | - | - | - | - |
| 7 | - | - | - | - |

O modo “Follow Lines” permite que o VANT siga a linha de transmissão, utilizando os valores retornados pelo serviço de detecção. O modo “Land In Line”, por sua vez, propicia o pouso sobre a linha central do segmento de linha de transmissão que o VANT está sobrevoando assim que este modo é habilitado.

### Serviço de detecção

Conforme mencionado, o nó cliente faz uma requisição de serviço ao nó servidor, que por sua vez retorna as informações requisitadas ao cliente. O serviço utilizado, definido por Detection.srv, possui os seguintes campos:
    
    • mode: variável de entrada (ou requisição) do serviço, do tipo uint8;
    
    • ld: variável de saída (ou resposta) do serviço, do tipo uint8;
    
    • th: variável de saída (ou resposta) do serviço, do tipo float32;
    
    • rh: variável de saída (ou resposta) do serviço, do tipo float32;
    
    • dy: variável de saída (ou resposta) do serviço, do tipo float32;

A requisição do cliente ocorre nos modos “Follow Lines” (5) e “Land In Line” (6). Os tipos de requisição são divididos em três, “full_line_mode”, “measure_line_mode” e “one_line_mode”. Basicamente, o tipo “full_line_mode” (1) realiza a detecção da linha de transmissão utilizando o sistema YOLO e retorna o número de linhas identificadas (ld), o ângulo dessas linhas em relação ao VANT (th), a altura relativa dessas linhas em relação ao VANT (rh) e a descentralização destas linhas em relação à imagem capturada pelo VANT (dy). Neste modo, os dois últimos valores, rh e dy, possuem uma acurácia menor e são mais adequados ao modo “Follow Lines”, onde é permitido um valor moderado de erro nos cálculos da altura relativa e descentralização.

O tipo “measure_line_mode” (2) é utilizado dentro do modo “Land In Line”. Com o VANT alinhado em relação à linha, em uma altura relativa de aproximadamente 15 metros e moderadamente centralizado, o serviço de detecção é chamado. Neste tipo, o cálculo da altura relativa é realizado de  maneira mais precisa, diminuindo o erro em até 33% conforme os testes realizados. Consequentemente, a descentralização do drone também é calculada de forma mais precisa.

O tipo “one_line_mode” (3) é utilizado no último estágio do modo “Land In Line”. Neste estágio, o VANT se encontra a aproximadamente 5 metros da linha central (considerando uma linha de transmissão de 230kV de circuito simples). Neste tipo de detecção, o YOLO não é utilizado. Em seu lugar são usados filtros e outras funções pré-definidas do OpenCV para calcular a descentralização da linha central em relação à imagem.

Para todos os tipos de serviço de detecção, o drone utiliza imagens recebidas do tópico /iris_fpv_cam/usb_cam/image_raw, que por sua vez são armazenadas em uma variável local. A tabela a seguir resume os tipos de serviço de detecção. A resposta “Sim” indica que o serviço retorna a informação avaliada:

| mode | ld | th | rh | dy |
|:-:|:-:|:-:|:-:|:-:|
| full_line_mode | Sim | Sim | Sim | Sim |
| measure_line_mode | - | - | Sim | - |
| one_line_mode | - | - | - | Sim |

## Dúvidas?

Entre em contato! lucas.diniz@engenharia.ufjf.br
