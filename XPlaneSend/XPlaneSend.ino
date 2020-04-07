/*
Sistema de ENVIO de mensagens para o X-Plane via UDP.
Desenvolvido por Judenilson Araujo Silva, www.judenilson.com.br
para uso no simulador de voo X-Plane, qualquer dúvida ou melhoramento, favor enviar pelo meio mais conveniente.
O uso é irrestrito e totalmente Open Source, você pode fazer o que quiser com o código, mas, se melhorar não deixe de informar por gentileza.
No entanto, ajude a difundir o código aberto fazendo o mesmo, se vc recebeu de graça, dê de graça também, ou pelo menos por um preço justo.
Abraço
*/

#include <WiFi.h>
#include "Arduino.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> 

const char* ssid     = "Judenilson_2.4"; //Colocar aqui o SSID do WiFi
const char* password = "judenilson82"; //Senha do wifi

// definindo GPIO dos _potenciômetros
const int _sensorPinA = A0;  // speedbrake
const int _sensorPinB = A3;  // throttle 1
const int _sensorPinC = A6;  // throttle 2
const int _sensorPinD = A7;  // propeller
const int _sensorPinE = A4;  // mixture
const int _sensorPinF = A5;  // flaps
// variáveis para guardar o valor do _potenciômetro
int _sensorValueA = 0, _sensorValueSaveA = 0;
int _sensorValueB = 0, _sensorValueSaveB = 0;
int _sensorValueC = 0, _sensorValueSaveC = 0;
int _sensorValueD = 0, _sensorValueSaveD = 0;
int _sensorValueE = 0, _sensorValueSaveE = 0;
int _sensorValueF = 0, _sensorValueSaveF = 0;
const int _correcaoPot = 0;	// variação para correção do ruido no _potenciômetro

IPAddress ipMulticast ( 239,255,1,1 ); // Broadcast do XP, será usado para saber qual ip oXP tá instalado
const int portMulticast = 49707; //Porta do broadcast do XP
WiFiUDP _Udp;	//instanciando o objeto de conexão.

char _packetBuffer[500]; //Buffer onde se obtém a informação vinda do XP
int _packetSize; // tamanho da mensagem recebida
uint8_t _bufferEnvio[413]; //buffer de envio para o XP
int _longBuffer;

// Estruturas de dados para armazenamento de insformações
struct DATA {
  char comando[4];
  char nulo;
  char data[495];
};

struct DATABECN {
  char comando[4];
  char nulo;
  char beacon_major_version[1];
  char beacon_minor_version[1];
  char application_host_id[4];
  char version_number[4];
  char role[4];
  char port[2];
  char computer_name[500];
};

struct INFOXPLANE {
  unsigned int beacon_major_version;
  unsigned int beacon_minor_version;
  unsigned int application_host_id;
  unsigned int version_number;
  unsigned int role;
  unsigned int port;
  char computer_name[500];
  IPAddress Ip;
};

DATA _receivedData;
DATABECN _dataBecn;
INFOXPLANE _infoXplane;

void setup() { // SETUP ----------------------------------------------------------------------------------------------------------------------------
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connectando wifi");

  WiFi.mode(WIFI_STA); //setando o wifi p conectar num roteador wifi
  WiFi.begin(ssid, password); //inicializando o wifi

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  infoXplane(WiFi.localIP()); // Metodo para conectar no X-Plane a primeira vez e pegar alguns dados.

  _Udp.begin(_infoXplane.port); 
}

void loop(){ 
	_sensorValueA = analogRead(_sensorPinA); 
  _sensorValueB = analogRead(_sensorPinB); 
  _sensorValueC = analogRead(_sensorPinC); 
  _sensorValueD = analogRead(_sensorPinD); 
  _sensorValueE = analogRead(_sensorPinE); 
  _sensorValueF = analogRead(_sensorPinF); 
  float _potValue = 0.0;
  
  // Serial.print(_sensorValueA); Serial.print(" - ");
  // Serial.print(_sensorValueB); Serial.print(" - ");
  // Serial.print(_sensorValueC); Serial.print(" - ");
  // Serial.print(_sensorValueD); Serial.print(" - ");
  // Serial.print(_sensorValueE); Serial.print(" - ");
  // Serial.println(_sensorValueF);

  if ((_sensorValueA < _sensorValueSaveA-_correcaoPot) || (_sensorValueA > _sensorValueSaveA+_correcaoPot)){
  		int value = map(_sensorValueA,800,2700,255,0);
  		_potValue = (float)value/256;
  		if (_potValue <= 0){ _potValue = 0.0; }
  		if (_potValue >= 1){ _potValue = 1.0; } 
    	writeDataRef(_potValue,"sim/multiplayer/controls/speed_brake_request[0]");
    	_sensorValueSaveA = _sensorValueA;
	}
  if ((_sensorValueB < _sensorValueSaveB-_correcaoPot) || (_sensorValueB > _sensorValueSaveB+_correcaoPot)){
  		int value = map(_sensorValueB,500,2800,0,255);
  		_potValue = (float)value/256;
  		if (_potValue <= 0){ _potValue = 0.0; }
  		if (_potValue >= 1){ _potValue = 1.0; } 
    	writeDataRef(_potValue,"sim/flightmodel/engine/ENGN_thro[0]");	
    	_sensorValueSaveB = _sensorValueB;
	}
  if ((_sensorValueC < _sensorValueSaveC-_correcaoPot) || (_sensorValueC > _sensorValueSaveC+_correcaoPot)){
  		int value = map(_sensorValueC,500,2800,0,255);
  		_potValue = (float)value/256;
  		if (_potValue <= 0){ _potValue = 0.0; }
  		if (_potValue >= 1){ _potValue = 1.0; } 
    	writeDataRef(_potValue,"sim/flightmodel/engine/ENGN_thro[1]");
    	_sensorValueSaveC = _sensorValueC;
	}  
	if (_sensorValueB < 400){
  		if (_sensorValueB >= 200){ 
  			writeDataRef(1,"sim/flightmodel/engine/ENGN_propmode[0]");			
    	  writeDataRef(0.0,"sim/flightmodel/engine/ENGN_thro[0]");}
  		else { 
  			writeDataRef(3,"sim/flightmodel/engine/ENGN_propmode[0]");
    	  writeDataRef(1.0,"sim/flightmodel/engine/ENGN_thro[0]");
  		}    	
    	_sensorValueSaveB = _sensorValueB;
	}
  if (_sensorValueC < 400){
  		if (_sensorValueC >= 200){ 
  			writeDataRef(1,"sim/flightmodel/engine/ENGN_propmode[1]");
    	  writeDataRef(0.0,"sim/flightmodel/engine/ENGN_thro[1]");}
  		else { 
  			writeDataRef(3,"sim/flightmodel/engine/ENGN_propmode[1]");
    	  writeDataRef(1.0,"sim/flightmodel/engine/ENGN_thro[1]");
  		}    	
    	_sensorValueSaveC = _sensorValueC;
	}
  if ((_sensorValueD < _sensorValueSaveD-_correcaoPot) || (_sensorValueD > _sensorValueSaveD+_correcaoPot)){
  	  	int value = map(_sensorValueD,500,2800,115,270);
  		_potValue = value;
  		if (_potValue <= 115){ _potValue = 115; }
  		if (_potValue >= 270){ _potValue = 270; } 
    	writeDataRef(_potValue,"sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec_all");
    	_sensorValueSaveD = _sensorValueD;
	}
  if ((_sensorValueE < _sensorValueSaveE-_correcaoPot) || (_sensorValueE > _sensorValueSaveE+_correcaoPot)){
  		int value = map(_sensorValueE,500,2800,0,255);
  		_potValue = (float)value/256;
  		if (_potValue <= 0){ _potValue = 0.0; }
  		if (_potValue >= 1){ _potValue = 1.0; } 
    	writeDataRef(_potValue,"sim/flightmodel/engine/ENGN_mixt[0]");
    	writeDataRef(_potValue,"sim/flightmodel/engine/ENGN_mixt[1]");
    	_sensorValueSaveE = _sensorValueE;
	}
  if ((_sensorValueF < _sensorValueSaveF-_correcaoPot) || (_sensorValueF > _sensorValueSaveF+_correcaoPot)){
  		int value = map(_sensorValueF,500,2800,255,0);
  		_potValue = (float)value/256;
  		if (_potValue <= 0){ _potValue = 0.0; }
  		else if (_potValue > 0 && _potValue <= 0.125){ _potValue = 0.125; }
  		else if (_potValue > 0.125 && _potValue <= 0.250){ _potValue = 0.250; }
  		else if (_potValue > 0.250 && _potValue <= 0.375){ _potValue = 0.375; }
  		else if (_potValue > 0.375 && _potValue <= 0.500){ _potValue = 0.500; }
  		else if (_potValue > 0.500 && _potValue <= 0.625){ _potValue = 0.625; }
  		else if (_potValue > 0.625 && _potValue <= 0.750){ _potValue = 0.750; }
  		else if (_potValue > 0.750 && _potValue <= 0.875){ _potValue = 0.875; }
  		else { _potValue = 1.0; }  
    	writeDataRef(_potValue,"sim/cockpit2/controls/flap_ratio");
    	_sensorValueSaveF = _sensorValueF;
	}
}

// método para enviar as datarefs e valores no simulador.
void writeDataRef(float value, String dataref) {
  _longBuffer = 509;
  memset (_bufferEnvio, 0, sizeof(505));
  memcpy (&_bufferEnvio, "DREF\0", 5);
  memcpy (&_bufferEnvio[5], &value, 4);
  for (int x = 0;x < _longBuffer; x++) {    
    if (x < dataref.length()){
    	_bufferEnvio[9+x] = dataref[x];
    } else if (x == dataref.length()){  		
    	_bufferEnvio[9+x] = '\0';
    } else{
    	_bufferEnvio[9+x] = ' ';
    }
  }    
	_Udp.beginPacket(_infoXplane.Ip, _infoXplane.port);
	_Udp.write(_bufferEnvio, _longBuffer);
	_Udp.endPacket();
	delay(7);
}

// método inicial que verifica os dados e a comunicação com o simulaor.
void infoXplane(IPAddress localIp) {
  bool infoOk = false;
  _Udp.beginMulticast(ipMulticast , portMulticast);
  while (!infoOk) {
    bool received = false;

	  while (!received) {
	    _packetSize = _Udp.parsePacket();
	    if (_packetSize) {
	      _Udp.read(_packetBuffer, _packetSize);
	      received = true;
	    }
	    delay(10);
	  }	  

    memcpy (&_receivedData, &_packetBuffer, _packetSize);

  	//Verificando se as informações do XP foram recebidas.
    if (strcmp(_receivedData.comando,"BECN")==0) {
      infoOk = true;
      memcpy (&_dataBecn, &_packetBuffer, _packetSize);
      memcpy (&_infoXplane.beacon_major_version, &_dataBecn.beacon_major_version, 1);
      memcpy (&_infoXplane.beacon_minor_version, &_dataBecn.beacon_minor_version, 1);
      memcpy (&_infoXplane.application_host_id, &_dataBecn.application_host_id, 4);
      memcpy (&_infoXplane.version_number, &_dataBecn.version_number, 4);
      memcpy (&_infoXplane.role, &_dataBecn.role, 4);
      memcpy (&_infoXplane.port, &_dataBecn.port, 2);
      memcpy (&_infoXplane.computer_name, &_dataBecn.computer_name, 500);
      _infoXplane.Ip = _Udp.remoteIP();
    }
  }
  _Udp.stop(); //fechando o multicast!
}

/*
BECN - Transmissão de beacon. Não há recurso equivalente para isso no ExtPlane. 
Toda instância em execução do X-Plane transmitirá seu endereço IP e número da porta de comando na sua LAN em uma mensagem BECN, a cada segundo. 
Com isso, é possível que os dispositivos na sua LAN "descubram" automaticamente o IP e o número da porta do servidor X-Plane. 
Também é possível detectar automaticamente se existem vários servidores X-Plane em execução na sua LAN e oferecer ao usuário a escolha de qual deles eles desejam que o dispositivo se conecte. 
É muito melhor do que o usuário configurar manualmente o dispositivo com o número de porta e IP do servidor e ajuda muito quando você tem mais de um PC X-Plane. 
Observe que este é um datagrama multicast, portanto, você precisará "assinar" a porta 49707 do endereço 239.255.1.1 do multicast.
RREF - Solicita assinatura para um DataRef. 
O cliente envia esta mensagem para a porta de comando do X-Plane, juntamente com uma frequência (em Hz) e um número de identificação. 
O X-Plane enviará automaticamente ao cliente uma mensagem RREF de resposta na frequência especificada, contendo o valor atual desse DataRef. 
Para cancelar a inscrição, você envia o RREF novamente com uma frequência de 0.
DREF - Defina o valor de um DataRef. Você envia um DREF junto com o valor desejado e o nome do dataref.
CMND - Pega o X-Plane para executar um comando. Você envia um CMND junto com o nome do comando que deseja chamar e o X-Plane o executa.
*/
