/* ESP32-CAM AI Thinker / OV2640 / MB / L298N / quatro motores.
   Compilar para AI Thinker ESP32-CAM, pacote Espressif 3.3.12.
   V9.5: curvas prioritarias, velocidade menor e captura de baixa latencia.
   Nenhum Wi-Fi/Bluetooth/SD, nem bibliotecas externas.
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "Config.h"
#include "Seguidor.h"
#include "Motores.h"
#include "Partida.h"
#include "CameraPreview.h"
#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "Selecione AI Thinker ESP32-CAM (ESP32 classico)."
#endif
seg::Detector detector;
seg::Controller controller;
partida::StartGate startGate;
preview::Recovery previewRecovery;
uint32_t waitSecondsLogged=0;
bool fatal=false, countdown=false;
uint32_t armAt=0, logAt=0, testAt=0, testDuration=0;
int validFrames=0,testLeft=0,testRight=0;
bool asciiRequested=false,pgmRequested=false;
char input[64];
size_t inputLength=0;
bool inputOverflow=false;
// RAM interna: devolve o framebuffer antes da análise e evita leituras
// repetidas da PSRAM. Uma única captura no driver, própria para GRAYSCALE.
static uint8_t visionImage[cfg::W*cfg::H];
bool initCamera(){
  if(!psramFound()){
    Serial.println("FALHA: PSRAM ausente. Selecione AI Thinker ESP32-CAM.");
    return false;
  }
  camera_config_t c={};
  c.ledc_channel=LEDC_CHANNEL_0;
  c.ledc_timer=LEDC_TIMER_0;
  c.pin_d0=5;
  c.pin_d1=18;
  c.pin_d2=19;
  c.pin_d3=21;
  c.pin_d4=36;
  c.pin_d5=39;
  c.pin_d6=34;
  c.pin_d7=35;
  c.pin_xclk=0;
  c.pin_pclk=22;
  c.pin_vsync=25;
  c.pin_href=23;
  c.pin_sccb_sda=26;
  c.pin_sccb_scl=27;
  c.pin_pwdn=32;
  c.pin_reset=-1;
  c.xclk_freq_hz=20000000;
  // O driver entrega luminancia em 8 bits: imagem ja em escala de cinza.
  c.pixel_format=PIXFORMAT_GRAYSCALE;
  c.frame_size=FRAMESIZE_QQVGA;
  c.fb_count=2;
  c.fb_location=CAMERA_FB_IN_PSRAM;
  c.grab_mode=CAMERA_GRAB_LATEST;
  c.jpeg_quality=12;
  esp_err_t err=esp_camera_init(&c);
  if(err!=ESP_OK){
    Serial.printf("FALHA camera: 0x%x\n",err);
    return false;
  }
  sensor_t *s=esp_camera_sensor_get();
  if(!s || s->id.PID!=OV2640_PID){
    Serial.println("FALHA: sensor diferente de OV2640.");
    return false;
  }
  if(s->set_hmirror(s,cfg::ESPELHAR_CAMERA)!=0 ||
      s->set_vflip(s,cfg::VIRAR_CAMERA_VERTICAL)!=0)return false;
  // Mantem exposicao automatica; iluminar melhor se o periodo de frame ficar alto.
  for(int i=0;i<12;++i){
    camera_fb_t *f=esp_camera_fb_get();
    if(!f)return false;
    esp_camera_fb_return(f);
    delay(25);
  }
  return true;
}
void stopAll(){
  startGate.cancel();
  waitSecondsLogged=0;
  countdown=false;
  testDuration=0;
  controller.stop();
  motores::command(0,0);
  validFrames=0;
  detector.reset();
}
void help(){
  Serial.println("BANCADA, 115200 baud, final Nova linha. Comandos:");
  Serial.println("s = iniciar apos 5s e linha estavel; x = parar/cancelar");
  Serial.println("v = imagem ASCII parado; g = imagem PGM parado (monitor Python/HTML)");
  Serial.println("p ESQ DIR MS = teste PWM assinado, ex.: p 140 140 700");
  Serial.println("Teste: |PWM| <= PWM_MAX, tempo 100..1500ms. h = ajuda");
}
void processCommand(){
  if(strcmp(input,"x")==0){
    stopAll();
    motores::clear();
    Serial.println("PARADO");
    return;
  }
  if(strcmp(input,"h")==0){
    help();
    return;
  }
  if(strcmp(input,"s")==0){
    if(motores::snapshot().fault){
      Serial.println("FALHA latente: x e revise alimentacao/camera.");
      return;
    }
    stopAll();
    countdown=true;
    armAt=millis();
    Serial.println("AGUARDANDO PARTIDA e 10 imagens com linha.");
    return;
  }
  if(controller.status()!=seg::State::STOP || countdown || testDuration){
    Serial.println("Pare com x antes de configurar/testar.");
    return;
  }
  if(strcmp(input,"v")==0){
    asciiRequested=true;
    return;
  }
  if(strcmp(input,"g")==0){
    pgmRequested=true;
    return;
  }
  int l,r,ms;
  char extra;
  if(sscanf(input,"p %d %d %d %c",&l,&r,&ms,&extra)==3 &&
      l>=-cfg::PWM_MAX&&l<=cfg::PWM_MAX&&r>=-cfg::PWM_MAX&&r<=cfg::PWM_MAX&&ms>=100&&ms<=1500){
    if(motores::snapshot().fault){
      Serial.println("Use x para limpar falha antes do teste.");
      return;
    }
    testLeft=l;
    testRight=r;
    testDuration=ms;
    testAt=millis();
    return;
  }
  Serial.println("Comando invalido. h = ajuda.");
}
void serialCommands(){
  while(Serial.available()){
    char c=(char)Serial.read();
    if(cfg::COMPETICAO)continue;
    // firmware de prova nao aceita comando externo
    if(c=='x'){
      stopAll();
      motores::clear();
      inputLength=0;
      inputOverflow=false;
      continue;
    }
    if(c=='\r')continue;
    if(c=='\n'){
      input[inputLength]=0;
      if(inputLength&&!inputOverflow)processCommand();
      else if(inputOverflow)Serial.println("Comando longo demais.");
      inputLength=0;
      inputOverflow=false;
    }
    else if(inputLength<sizeof(input)-1)input[inputLength++]=c;
    else inputOverflow=true;
  }
}
void showFrame(const uint8_t *pixels,const seg::Vision &v){
  bool transmitting=asciiRequested||pgmRequested;
  if(asciiRequested){
    asciiRequested=false;
    Serial.println("# = escuro; . = claro. Fita deve aparecer embaixo e no meio:");
    for(int y=0;y<cfg::H;y+=4){
      for(int x=0;x<cfg::W;x+=2)
        Serial.write(pixels[y*cfg::W+x]<v.threshold?'#':'.');
      Serial.println();
    }
  }
  if(pgmRequested){
    pgmRequested=false;
    Serial.print("\nFRAME_PGM_BEGIN\nP5\n160 120\n255\n");
    Serial.write(pixels,cfg::W*cfg::H);
    Serial.print("\nFRAME_PGM_END\n");
  }
  if(transmitting)previewRecovery.sent();
}
void setup(){
  // GPIOs ainda nao sao controlados durante ROM/boot: ver resistores no guia.
  for(int p:motores::pins){
    pinMode(p,OUTPUT);
    digitalWrite(p,LOW);
  }
  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);
  // flash apagado
  Serial.setTxBufferSize(512);
  // comporta a telemetria sem esperar o USB
  Serial.begin(115200);
  Serial.printf("\nCarrinho V9.5 CURVAS + CAMERA RAPIDA / reset=%d / modo=%s\n",(int)esp_reset_reason(),cfg::COMPETICAO?"PROVA":"BANCADA");
  if(!motores::begin()){
    Serial.println("FALHA PWM/tarefa motores.");
    fatal=true;
    return;
  }
  if(!initCamera()){
    motores::fault();
    fatal=true;
    return;
  }
  if(cfg::COMPETICAO){
    // Todo boot inicia uma nova tentativa, desde que camera e motores inicializem.
    detector.reset();
    startGate.begin(millis());
    Serial.println("PROVA: tentativa 1, procurando a linha por ate 5s.");
  }
  else help();
}
void loop(){
  if(fatal){
    delay(100);
    return;
  }
  serialCommands();
  uint32_t cycleStart=millis();
  camera_fb_t *f=esp_camera_fb_get();
  int64_t nowUs=esp_timer_get_time();
  uint32_t frameAt=0;
  int64_t frameUs=0;
  bool good=f && f->format==PIXFORMAT_GRAYSCALE && f->width==cfg::W && f->height==cfg::H && f->len>=cfg::W*cfg::H;
  if(good){
    frameUs=(int64_t)f->timestamp.tv_sec*1000000+f->timestamp.tv_usec;
    frameAt=(uint32_t)(frameUs/1000);
  }
  motores::Mailbox beforeFrame=motores::snapshot();
  bool parked=!cfg::COMPETICAO && controller.status()==seg::State::STOP &&
    !countdown && !testDuration && !startGate.active() && !beforeFrame.left && !beforeFrame.right &&
    !beforeFrame.actualL && !beforeFrame.actualR;
  preview::Frame decision=previewRecovery.inspect(good,frameUs,nowUs,parked);
  if(decision==preview::Frame::DROP){
    esp_camera_fb_return(f);
    delay(1);
    return;
    // sem analisar nem mover com imagem antiga
  }
  if(decision==preview::Frame::FAULT){
    if(f)esp_camera_fb_return(f);
    stopAll();
    controller.fault();
    motores::fault();
    Serial.println("FALHA: camera sem frame valido/recente.");
    delay(50);
    return;
  }
  memcpy(visionImage,f->buf,sizeof(visionImage));
  esp_camera_fb_return(f);
  uint32_t visionStart=micros();
  seg::Vision v=detector.analyze(visionImage,sizeof(visionImage));
  uint32_t visionUs=micros()-visionStart;
  if(controller.status()==seg::State::STOP&&!countdown&&!testDuration&&!startGate.active())showFrame(visionImage,v);
  uint32_t now=millis();
  if(v.valid && fabsf(v.nearX-cfg::CENTRO_CAMERA)<35)validFrames=validFrames<100?validFrames+1:100;
  else validFrames=0;
  if(motores::snapshot().fault){
    controller.fault();
    countdown=false;
    testDuration=0;
    startGate.cancel();
  }
  if(cfg::COMPETICAO && startGate.active()){
    auto before=startGate.state();
    bool aligned=v.valid && fabsf(v.nearX-cfg::CENTRO_CAMERA)<35;
    if(startGate.observe(now,frameAt,aligned)){
      controller.begin(now);
      Serial.println("PARTIDA: linha confirmada, iniciando percurso.");
    }
    else{
      if(startGate.state()==partida::State::WAITING){
        uint32_t seconds=(startGate.remainingMs(now)+999)/1000;
        if(seconds!=waitSecondsLogged){
          waitSecondsLogged=seconds;
          Serial.printf("SEM LINHA: parado, nova tentativa em %lus.\n",(unsigned long)seconds);
        }
      }
      else if(before!=startGate.state()){
        waitSecondsLogged=0;
        detector.reset();
        Serial.printf("PROVA: tentativa %lu, procurando por ate 5s.\n",(unsigned long)startGate.attempt());
      }
    }
  }
  if(countdown && seg::elapsed(now,armAt)>=cfg::ESPERA_PARTIDA_MS && validFrames>=10){
    countdown=false;
    controller.begin(now);
    Serial.println("PARTIDA");
  }
  seg::Drive d;
  if(startGate.active()){
    // Ao ligar nao existe uma ultima posicao confiavel: somente observa.
    motores::command(0,0,seg::Pace::CONTINUOUS,frameAt);
  }
  else if(testDuration){
    if(seg::elapsed(now,testAt)>=testDuration){
      testDuration=0;
      motores::command(0,0);
    }
    else motores::command(testLeft,testRight);
  }
  else{
    d=controller.step(v,now);
    seg::PwmDrive pwm=seg::pwmForDrive(d);
    motores::command(pwm.left,pwm.right,d.pace,frameAt,d.holdMs);
  }
  if(seg::elapsed(now,logAt)>=250){
    logAt=now;
    motores::Mailbox m=motores::snapshot();
    char line[240];
    int n=snprintf(line,sizeof(line),"%s T=%d C=%d faixas=%d perto=%.0f erro=%.2f X=%d E=%d D=%d passo=%d ciclo=%lums visao=%luus idade=%lums curva=%d y=%d risco=%.2f falha=%d\n",
        startGate.active()?(startGate.state()==partida::State::LOOKING?"PROCURANDO":"ESPERA_3S"):(d.state==seg::State::SEARCH && (d.left<0 || d.right<0)?"RETORNO":seg::name(controller.status())),v.threshold,v.contrast,v.count,v.nearX,v.error,v.junction,
        m.actualL,m.actualR,m.pulseOn,(unsigned long)seg::elapsed(now,cycleStart),
        (unsigned long)visionUs,(unsigned long)seg::elapsed(now,frameAt),v.cornerDirection,v.cornerY,v.curve,m.fault);
    // Telemetria nunca segura o laço de visao esperando o USB.
    if(n>0&&n<(int)sizeof(line)&&Serial.availableForWrite()>=n)Serial.write((const uint8_t*)line,n);
  }
  // Nao adiciona atraso artificial: esp_camera_fb_get() ja sincroniza
  // naturalmente com a chegada do proximo quadro.
}
