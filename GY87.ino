/*
    Script feito para ler todos os dados dos 3 sensores da placa GY87
      - Acelerômetro
      - Giróscopio
      - Magnetómetro
    Usando a biblioteca do MPU6050 é possível ter acesso a DMP e suas propriedades
    dessa forma, a aceleração e velocidade angular do sensor já são filtrados e um
    quanternions pode ser obtido através desses dois sensroes.
    Como o magnetômetro não está no mesmo encapsulamento não foi possível enviar os dados
    sobre os campos magnéticos para a DMP do MPU6050

    Conexões com o arduino: 3.3V - 3.3V
                            GND - GND
                            SDA - SDA
                            SCL - SCL
                            INT - 0

    Autor: Eduardo Morais Carvalho
           Graduação em Engenharia Biomédica - UFU
           Trabalho de Conclusão de Curso
    Tutor: Dr. Prof. Alcimar Barbosa Soares
*/

// Incluir as bilbiotecas necessárias
#include "helper_3dmath.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "Timer.h"

// A biblioteca Wire é necessária caso I2Cdev I2CDEV_ARDUINO_WIRE seja necessário
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// LED do arduino para piscar durante a aquisição
#define LED_PIN 13
bool blinkState = false;

// Criar instâncias dos sensores para poder manipula-los
MPU6050 mpu;
HMC5883L hmc5883l;

// Para a aquisição de dados será usado um timer
Timer timer;

// Variavéis de controle e status da MPU
int interruptPin = 0;   // Pino para gerar interrupção quando necessário
bool dmpReady = false;  // Flag para saber se a DMP foi configurada e está pronta
uint8_t mpuIntStatus;   // Flag para saber o estado no PINO de interrupção
uint8_t devStatus;      // Retorna o estado depois de cada loop de aquisição (0 = success, !0 = error)
uint16_t packetSize;    // tamanho esperado da DMP de acordo com o datasheet (padrão é 42 bytes)
uint16_t fifoCount;     // Estado atual de quantos bytes estão na FIFO
uint8_t fifoBuffer[64]; // Memória FIFO

// Variavéis de orientação
Quaternion q;           // [w, x, y, z]  Segura o valor do quaternion
VectorInt16 mag;        // [x, y, z]     Magnetômetro
VectorInt16 gyro;       // [x, y, z]     Giróscopio
VectorInt16 aa;         // [x, y, z]     Acelerômetro
VectorInt16 aaReal;     // [x, y, z]     Acelerômetro sem a influência da gravidade
VectorInt16 aaWorld;    // [x, y, z]     Acelerômetro transladado para o eixo de rotação da terra
VectorFloat gravity;    // [x, y, z]     Vetor gravidade

// Rotina de interrupção
// Indica se o pino de interrupção está ativado
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

// Rotina de controle para abrir a comunicação com o sensor escravo (HMC5883L)
// ou qualquer outro sensor que que seja escravo em relação ao MPU6050
void setSlaveControl(uint8_t slaveID)
{
  mpu.setSlaveEnabled(slaveID, true);
  mpu.setSlaveWordByteSwap(slaveID, false);
  mpu.setSlaveWriteMode(slaveID, false);
  mpu.setSlaveWordGroupOffset(slaveID, false);
  mpu.setSlaveDataLength(slaveID, 2);
}

void setup()
{
  // Inicia o barramento I2C, se isso já não for feito automaticamente
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Inicia a comunicação serial e a velocidade de comunicação
  Serial.begin(250000);
  // Espera que o monitor serial, ou a comunicação serial comece
  while (!Serial);

  // Abre a comunicação com o MPU6050
  Serial.println(F("Inicializando a MPU6050..."));
  mpu.initialize();

  // Verifica se a conexão foi estabelecida
  Serial.println(F("Testando a conexao com a MPU..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 esta conectado") : F("MPU6050 esta desconectado"));

  // Com o sensor conectado já é possível configurar a DMP do MPU6050
  Serial.println(F("Configurando a DMP..."));
  devStatus = mpu.dmpInitialize();

  // Configurar o alcance e sensibilidade da MPU6050
  //mpu.setFullScaleAccelRange(3);
  //mpu.setFullScaleGyroRange(3);

  // Ajustar os offsets do MPU6050
  mpu.setXAccelOffset(571);
  mpu.setYAccelOffset(1582);
  mpu.setZAccelOffset(1933);
  mpu.setXGyroOffset(60);
  mpu.setYGyroOffset(-55);
  mpu.setZGyroOffset(13);

  // Iniciar a configuração do HMC5883L
  // Acessar o escravo HMC5883L usando o MPU6050 como mestre
  mpu.setI2CMasterModeEnabled(0);
  mpu.setI2CBypassEnabled(1);

  // Com a configuração completa do MPU6050, pode-se configurar o HMC5883L
  if (hmc5883l.testConnection())
  {
    // Se conexão foi estabelecida:
    Serial.println("HMC5883L conectado");

    // Comando para colocar HMC5883L em modo contínuo de leitura
    I2Cdev::writeByte(HMC5883L_DEFAULT_ADDRESS,
                      HMC5883L_RA_MODE,
                      HMC5883L_MODE_CONTINUOUS);

    // HMC5883L configurado, deve-se voltar a 'falar' com o mestre - MPU6050
    mpu.setI2CBypassEnabled(0);

    // Eixo X do HMC5883L
    mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
    setSlaveControl(0);

    // Eixo Y do HMC5883L
    mpu.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
    setSlaveControl(1);

    // Eixo Y do HMC5883L
    mpu.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
    setSlaveControl(2);
    mpu.setI2CMasterModeEnabled(1);

  } else
  {
    // Se nada deu certo durante a configuração
    Serial.println("HMC5883L nao foi configurado");
  }

  // Se até aqui a configuração deu certo
  if (devStatus == 0)
  {
    // Agora sim, ligar a DMP a MPU6050
    Serial.println(F("Ligando a DMP..."));
    mpu.setDMPEnabled(true);

    // Configurar o pino de interrupção do arduino com a MPU6050
    Serial.println(F("Configurando o pino de interrupcao..."));
    attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Configura a Flag da DMP, dessa forma o loop principal pode usar a DMP
    Serial.println(F("DMP pronta! Esperando a primeira interrupcao..."));
    dmpReady = true;

    // Salva o tamanho padrão DMP para comparações posteriores
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else
  {
    // Se aparecer um mensagem de ERRO!
    // 1 = a memória inicial falhou
    // 2 = a configuração da DMP falhou
    // (Normalmente o codigo 1 aparece)
    Serial.print(F("Erro de configuracao da DMP (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  // Configurar o LED
  pinMode(LED_PIN, OUTPUT);

  // Configurar o timer para aquisição de dados - 5ms ( Frequência de aquisição = 200Hz )
  timer.every(5, readSensor);
}

void loop()
{
  // A cada vez que o tick do timer vence, a função readSensor() é executado
  timer.update();
}

void readSensor()
{
  // Se a DMP não foi configurada corretamente
  if (!dmpReady) return;

  // Esperar enquanto a interrupção não for ativada ou nenhum pacote de dados chegar
  while (!mpuInterrupt && fifoCount < packetSize);

  // Quando chegar um novo valor, reinicar a flag de interrupção
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Pegar o tamanho atual da FIFO
  fifoCount = mpu.getFIFOCount();

  // Assegurar que a fifo não estore
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // Se estorar, os dados extras vão ser perdidos, mas o processo de aquisição ira continuar
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // Caso contrário, checar se a DMP estpa pronta
  } else if (mpuIntStatus & 0x02)
  {
    // Esperar que o tamanho do pacote que está chegando, seja o tamanho padrão da FIFO
    while (fifoCount < packetSize) 
      fifoCount = mpu.getFIFOCount();

    // Quando o tamanho for igual ao tamanho padrão da FIFO, ler a FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);


    // Acompanhar o tamanho da FIFO, caso algum valor novo seja gerado, ele já pode ser lido
    // sem esperar que a ocorrência de uma nova interrupção
    fifoCount -= packetSize;
    //mpu.getMotion6(&aa.x, &aa.y, &aa.z, &gyro.x, &gyro.y, &gyro.z);
    //mpu.dmpGet6AxisQuaternion(&q, fifoBuffer);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetGyro(&gyro, fifoBuffer);

    mag.x = mpu.getExternalSensorWord(0);
    mag.y = mpu.getExternalSensorWord(2);
    mag.z = mpu.getExternalSensorWord(4);
    //mpu.dmpGetExternalSensorData(&mag_test,2,fifoBuffer);
    Serial.print("raw\t");

    Serial.print(aaReal.x); Serial.print("\t");
    Serial.print(aaReal.y); Serial.print("\t");
    Serial.print(aaReal.z); Serial.print("\t");
/*
    Serial.print(gyro.x); Serial.print("\t");
    Serial.print(gyro.y); Serial.print("\t");
    Serial.print(gyro.z); Serial.print("\t");

    Serial.print(mag.x); Serial.print("\t");
    Serial.print(mag.y); Serial.print("\t");
    Serial.print(mag.z); Serial.print("\t");

    Serial.print(gravity.x); Serial.print("\t");
    Serial.print(gravity.y); Serial.print("\t");
    Serial.print(gravity.z); Serial.print("\t");
    */
/*
    Serial.print(q.w); Serial.print("\t");
    Serial.print(q.x); Serial.print("\t");
    Serial.print(q.y); Serial.print("\t");
    Serial.print(q.z); Serial.print("\t");
*/
    //Serial.print(mpu.getFullScaleAccelRange());
    Serial.println();
  }
}











