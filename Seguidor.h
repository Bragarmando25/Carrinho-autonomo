#pragma once
// Nucleo independente de Arduino: os testes executam este mesmo codigo no PC.
#include "Config.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
namespace seg {
  inline float limit(float x, float a, float b) {
    return (x < a) ? a : ((x > b) ? b : x);
  }
  inline int sign(float x) {
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
  }
  inline uint32_t elapsed(uint32_t now, uint32_t then) {
    return now - then;
  }
  struct Row {
    bool found = false;
    bool wide = false;
    float x = 0;
    float width = 0;
    float lo = 0;
    float hi = 0;
    int candidates = 0;
  };
  struct Vision {
    bool valid = false;
    bool junction = false;
    bool cornerNear = false;
    bool cornerAhead = false;
    int cornerDirection = 0;
    int cornerY = 0;
    // posição vertical; cresce quando o cotovelo se aproxima
    int threshold = 0;
    int contrast = 0;
    int count = 0;
    int nearCount = 0;
    int hint = 0;
    float nearX = cfg::CENTRO_CAMERA;
    float farX = cfg::CENTRO_CAMERA;
    float heading = 0;
    float error = 0;
    float confidence = 0;
    float curve = 0;
    // intensidade da curva à frente, de 0 a 1
    Row rows[cfg::N];
  };
  class Detector {
    float seed=cfg::CENTRO_CAMERA, incoming=0;
    bool history=false;
    int misses=0;
    static float expectedWidth(int y) {
      return cfg::LARGURA_LONGE+(cfg::LARGURA_PERTO-cfg::LARGURA_LONGE)*(y-24)/88.0f;
    }
    static int percentile(const int *hist, int n, float p) {
      int sum=0, target=(int)(n*p);
      if(target<1)target=1;
      for(int i=0;i<256;++i) {
        sum+=hist[i];
        if(sum>=target)return i;
      }
      return 255;
    }
    public:
    void reset() {
      seed=cfg::CENTRO_CAMERA;
      incoming=0;
      history=false;
      misses=0;
    }
    Vision analyze(const uint8_t *gray, size_t size) {
      Vision v;
      if(misses>=3){
        history=false;
        incoming=0;
      }
      if(misses<4)++misses;
      if(!gray || size<(size_t)cfg::W*cfg::H)return v;
      int hist[256]={0}, total=0;
      int sum=0;
      for (int y = cfg::ROI_TOPO_Y; y < 116; y += 4)
        for (int x = 1; x < 159; x += 4) {
        int a=gray[y*cfg::W+x];
        ++hist[a];
        ++total;
        sum+=a;
      }
      int lo=percentile(hist,total,0.015f), hi=percentile(hist,total,0.85f);
      v.contrast=hi-lo;
      if(v.contrast<cfg::MIN_CONTRASTE)return v;
      // Otsu valida separacao. O limiar usa percentis escuro/claro do piso,
      // evitando que rejunte e sombras elevem a media da classe escura.
      int nb=0, bestN=0, sb=0, bestSb=0;
      float best=-1;
      for(int t=0;t<255;++t) {
        nb+=hist[t];
        sb+=t*hist[t];
        if(nb<total/100 || total-nb<total/10)continue;
        float d=(float)sb/nb-(float)(sum-sb)/(total-nb);
        float score=(float)nb*(total-nb)*d*d;
        if(score>best){
          best=score;
          bestN=nb;
          bestSb=sb;
        }
      }
      if(bestN==0 || bestN==total)return v;
      float dark=(float)bestSb/bestN, light=(float)(sum-bestSb)/(total-bestN);
      if(light-dark<cfg::MIN_CONTRASTE)return v;
      v.threshold=cfg::LIMIAR_FIXO>=0?cfg::LIMIAR_FIXO:(int)(lo+cfg::FRACAO_LIMIAR_AUTO*(hi-lo));
      float predicted=history?seed:cfg::CENTRO_CAMERA;
      float slope=history?incoming:0, lastX=predicted;
      int lastY=cfg::Y[0], found=0, broad=0, narrow=0;
      int splitBefore=0, splitAfter=0;
      int nearNarrow=0, beyondStraight=0, cornerVotesLeft=0, cornerVotesRight=0;
      int cornerY=0;
      int bilateralWide=0;
      // faixa larga realmente cruza a trajetoria para os dois lados
      float nearSum=0, nearWeight=0, farSum=0, farWeight=0, midSum=0, midWeight=0;
      bool crossed=false;
      float crossingSlope=0,crossingX=0;
      int crossingY=0;
      for(int i=0;i<cfg::N;++i) {
        const int y=cfg::Y[i];
        if(y<cfg::ROI_TOPO_Y)continue;
        // Fundo local em 32 níveis: percentil suficiente para limitar o limiar,
        // com menos zeragem de memória e varredura que 256 níveis por faixa.
        uint16_t rowHist[32]={0};
        // Só estima o piso: amostra espaçada. A detecção da fita abaixo usa
        // TODOS os pixels, inclusive os da fita fina entre essas amostras.
        for(int x=1;x<159;x+=4)++rowHist[gray[y*cfg::W+x]>>3];
        int accumulated=0,background=252;
        for(int bin=0;bin<32;++bin){
          accumulated+=rowHist[bin];
          if(accumulated>=32){
            background=bin*8+4;
            break;
          }
        }
        // Se a faixa inteira e fita transversal, seu percentil nao e piso.
        if(background<dark+0.25f*(light-dark))background=(int)light;
        int threshold=v.threshold;
        if(cfg::LIMIAR_FIXO<0 && threshold>background-20)threshold=background-20;
        uint8_t mask[cfg::W]={0};
        // Mesma maioria 3x3, reutilizando colunas: 3 leituras/pixel em vez de 9.
        uint8_t columns[cfg::W];
        for(int x=0;x<cfg::W;++x)
          columns[x]=(gray[(y-1)*cfg::W+x]<threshold)+
            (gray[y*cfg::W+x]<threshold)+(gray[(y+1)*cfg::W+x]<threshold);
        for(int x=1;x<cfg::W-1;++x)
          mask[x]=(columns[x-1]+columns[x]+columns[x+1]>=5);
        for(int x=2;x<158;++x)if(!mask[x]&&mask[x-1]&&mask[x+1])mask[x]=1;
        if(found)predicted=crossed?crossingX+crossingSlope*(crossingY-y):lastX+slope*(lastY-y);
        predicted=limit(predicted,1,158);
        float bestScore=1e9f, chosen=0, width=0, chosenLo=0, chosenHi=0;
        bool chosenWide=false;
        float nominal=expectedWidth(y);
        for(int x=1;x<159;) {
          if(!mask[x]){
            ++x;
            continue;
          }
          int a=x;
          while(x<159&&mask[x])++x;
          int b=x-1, w=b-a+1;
          if(w<cfg::MIN_LARGURA)continue;
          // faixa transversal pode ocupar toda largura
          ++v.rows[i].candidates;
          // Fita inclinada ocupa mais pixels horizontalmente sem ser transversal.
          float projection=limit(sqrtf(1+slope*slope),1.0f,1.8f);
          bool wide=w>nominal*2.3f*projection;
          float c=wide?limit(predicted,(float)a,(float)b):(a+b)*0.5f;
          float distance=fabsf(c-predicted);
          // Permite seguir curva saindo pela lateral, mas nao saltar de pista.
          float gate=crossed?24.0f:(found?22.0f+0.8f*(lastY-y):(history?45.0f:80.0f));
          if(distance>gate)continue;
          float score=distance+0.45f*fabsf(w-nominal)+(wide?8.0f:0);
          if(score<bestScore) {
            bestScore=score;
            chosen=c;
            width=w;
            chosenWide=wide;
            chosenLo=a;
            chosenHi=b;
          }
        }
        Row &r=v.rows[i];
        if(bestScore==1e9f)continue;
        // Num X diagonal, dois ramos se unem antes de se separar outra vez.
        // Essa união pode ser menor que uma fita transversal de 90 graus.
        if(!crossed && splitBefore>=2 && r.candidates==1 && width>nominal*1.45f){
          chosenWide=true;
          chosen=limit(predicted,chosenLo,chosenHi);
        }
        r.found=true;
        r.x=chosen;
        r.width=width;
        r.wide=chosenWide;
        r.lo=chosenLo;
        r.hi=chosenHi;
        if(r.candidates>1){
          if(crossed)++splitAfter;
          else ++splitBefore;
        }
        if(chosenWide){
          ++broad;
          if(!crossed){
            crossed=true;
            crossingSlope=slope;
            crossingX=chosen;
            crossingY=y;
          }
        }
        if(found && !chosenWide) {
          float measured=(chosen-lastX)/(lastY-y);
          // Ao atravessar mancha larga, conserva tangente de entrada no X.
          slope=crossed?crossingSlope:0.35f*slope+0.65f*limit(measured,-3,3);
        }
        lastX=chosen;
        lastY=y;
        ++found;
        if(!chosenWide)++narrow;
        if(y>=88 && !chosenWide)++nearNarrow;
        // Evidencia de saida reta apos uma faixa larga: identifica tambem o '+'.
        if(crossed && !chosenWide && fabsf(chosen-predicted)<18)++beyondStraight;
        // Uma ponta lateral longa sem a ponta oposta sugere cotovelo, nao X.
        if(chosenWide && y<=104){
          float toLeft=predicted-chosenLo, toRight=chosenHi-predicted;
          // Cruzamento real corta a trajetoria que o carrinho ja vinha seguindo:
          // deve haver fita transversal dos DOIS lados do caminho previsto.
          if(toLeft>24 && toRight>24)++bilateralWide;
          // Se a mancha larga cresce praticamente so para um lado, trate como
          // ponta de curva/cotovelo, nao como cruzamento.
          if(toRight>30 && toLeft<20){
            ++cornerVotesRight;
            if(y>cornerY)cornerY=y;
          }
          if(toLeft>30 && toRight<20){
            ++cornerVotesLeft;
            if(y>cornerY)cornerY=y;
          }
        }
        if(y>=88 && !chosenWide) {
          float weight=(y-80)/8.0f;
          nearSum+=chosen*weight;
          nearWeight+=weight;
          ++v.nearCount;
        }
        if(y<=80 && !chosenWide) {
          farSum+=chosen;
          farWeight+=1;
        }
        if(y<=80 && y>=56 && !chosenWide){
          float w=(y-48)/8.0f;
          midSum+=chosen*w;
          midWeight+=w;
        }
      }
      v.count=found;
      if(nearWeight)v.nearX=nearSum/nearWeight;
      if(farWeight)v.farX=farSum/farWeight;
      else v.farX=lastX;
      // Um cruzamento nao e apenas uma faixa larga. Curvas fechadas tambem
      // podem parecer largas na perspectiva. Exigimos um braco transversal
      // dos DOIS lados da trajetoria prevista + continuidade da pista principal.
      // Isso impede uma curva lateral de ser confundida com X/+.
      const bool bilateralCross = bilateralWide >= 1;
      const bool mainTrackContinues = nearNarrow >= 2 && beyondStraight >= 2;
      const bool splitCross = splitBefore >= 1 && splitAfter >= 1 && beyondStraight >= 1;

      const bool cornerToRight =
          cornerVotesRight > 0 && cornerVotesLeft == 0;
      const bool cornerToLeft =
          cornerVotesLeft > 0 && cornerVotesRight == 0;
      const bool unilateralCorner = cornerToRight || cornerToLeft;

      v.junction =
          broad >= 1 &&
          ((bilateralCross && mainTrackContinues) || splitCross);

      // Curva fechada pode parecer um cruzamento por alguns quadros.
      // Se a faixa larga cresce claramente para apenas um lado e nao existe
      // continuidade reta suficiente depois dela, a prioridade e da curva.
      if (unilateralCorner && beyondStraight < 2) {
        v.junction = false;
      }

      if (!v.junction && nearNarrow >= 2 && unilateralCorner) {
        v.cornerAhead = true;
        v.cornerNear = cornerY >= cfg::COTOVELO_PERTO_Y;
        v.cornerDirection = cornerToRight ? 1 : -1;
      }
      if(v.cornerAhead)v.cornerY=cornerY;
      if(v.junction && history && narrow>=3 && broad<=5 && !nearWeight){
        v.nearX=seed;
        v.nearCount=1;
        // centro previsto somente durante X reconhecido
      }
      // Cotovelo ja muito perto pode deixar so duas faixas estreitas visiveis.
      // Com evidencia da ponta lateral, ainda ha informacao para girar.
      v.valid=(narrow>=3 || (v.cornerNear && nearNarrow>=2)) && v.nearCount>=1;
      v.confidence=limit(narrow/16.0f,0,1);
      if(narrow>0)v.hint=sign(v.farX-cfg::CENTRO_CAMERA);
      if(v.cornerNear)v.hint=v.cornerDirection;
      if(v.valid) {
        misses=0;
        v.heading=limit((v.farX-v.nearX)/60.0f,-1.5f,1.5f);
        // Faixas intermediarias antecipam a curva sem deixar um S distante cancelar a proxima curva.
        if(midWeight && !v.junction && !v.cornerAhead){
          float localHeading=limit((midSum/midWeight-v.nearX)/32.0f,-1.5f,1.5f);
          v.heading=limit(0.70f*localHeading+0.30f*v.heading,-1.5f,1.5f);
          // Em S, a próxima curva pode apontar para um lado e a seguinte para
          // o outro. Usa o primeiro desvio consistente, sem fazer a média dos dois.
          int firstDirection=0,votes=0;
          float previousOffset=0;
          for(int j=0;j<cfg::N;++j){
            if(cfg::Y[j]>80)continue;
            const Row &r=v.rows[j];
            if(!r.found || r.wide){
              votes=0;
              continue;
            }
            float offset=r.x-v.nearX;
            if(fabsf(offset)<8){
              votes=0;
              continue;
            }
            int side=sign(offset);
            votes=side==firstDirection?votes+1:1;
            firstDirection=side;
            if(votes>=2){
              v.heading=limit((previousOffset+offset)/64.0f,-1.5f,1.5f);
              break;
            }
            previousOffset=offset;
          }
        }
        float lateral=v.nearX-cfg::CENTRO_CAMERA;
        // Quando a fita ja esta na borda, recupera primeiro a posicao proxima.
        float anticipation=cfg::PESO_DIRECAO*limit(1-fabsf(lateral)/60.0f,0.20f,1.0f);
        v.error=limit(lateral/79.5f+anticipation*v.heading,-1.5f,1.5f);
        // O trecho distante pede desaceleração; só cornerNear autoriza o giro.
        // Máximo por faixa evita que curvas opostas de um S se cancelem na média.
        if(!v.junction){
          for(int i=0;i<cfg::N;++i)
            if(cfg::Y[i]<=80 && v.rows[i].found && !v.rows[i].wide)
              v.curve=fmaxf(v.curve,limit(fabsf(v.rows[i].x-v.nearX)/45.0f,0,1));
          if(v.cornerAhead)v.curve=1;
        }
        if(!v.junction) {
          // Memoriza direcao das faixas proximas, nao o ramo distante do X.
          int a=-1,b=-1;
          for(int i=0;i<cfg::N && cfg::Y[i]>=88;++i)if(v.rows[i].found&&!v.rows[i].wide){
            if(a<0)a=i;
            b=i;
          }
          if(a>=0)seed=v.rows[a].x;
          if(b>a)incoming=limit((v.rows[b].x-v.rows[a].x)/(cfg::Y[a]-cfg::Y[b]),-2,2);
          history=true;
        }
      }
      return v;
    }
  };
  enum class State {
    STOP, FOLLOW, CROSS, PIVOT, SEARCH, FAULT
  };
  enum class Pace {
    CONTINUOUS, STRAIGHT, CAREFUL, CROSSING, TURN, RECOVERY
  };
  inline bool usesPulses(Pace p){
    if(p==Pace::CONTINUOUS)return false;
    return cfg::PASSOS_ATIVOS || p==Pace::RECOVERY ||
      (cfg::CURVAS_EM_PASSOS && (p==Pace::CAREFUL || p==Pace::TURN));
  }
  inline uint32_t onTime(Pace p){
    if(p==Pace::RECOVERY)return cfg::REENCONTRO_MOVE_MS;
    return p==Pace::STRAIGHT?cfg::RETA_MOVE_MS:(p==Pace::TURN?cfg::GIRO_MOVE_MS:
        (p==Pace::CROSSING?cfg::CRUZ_MOVE_MS:cfg::CURVA_MOVE_MS));
  }
  inline uint32_t offTime(Pace p){
    if(p==Pace::RECOVERY)return cfg::REENCONTRO_PAUSA_MS;
    return p==Pace::STRAIGHT?cfg::RETA_PAUSA_MS:(p==Pace::TURN?cfg::GIRO_PAUSA_MS:
        (p==Pace::CROSSING?cfg::CRUZ_PAUSA_MS:cfg::CURVA_PAUSA_MS));
  }
  // Executado pela tarefa dos motores. O tempo de passo nao depende do FPS.
  class PulseGate {
    enum Phase {
      IDLE, MOVING, REST
    }
    phase=IDLE;
    uint32_t since=0, budget=0, rest=0, boundaryGeneration=0;
    public:
    void reset(){
      *this=PulseGate();
    }
    bool tick(uint32_t now,uint32_t generation,uint32_t on,uint32_t off,bool requested,uint32_t sampleAgeMs=0){
      if(phase==IDLE){
        if(!requested)return false;
        phase=MOVING;
        since=now;
        budget=on;
        rest=off;
        return true;
      }
      if(phase==MOVING){
        // Uma pista mais dificil encurta o passo atual, nunca o prolonga.
        if (on < budget) budget = on;
        if (off > rest) rest = off;
        if(!requested || elapsed(now,since)>=budget){
          phase=REST;
          since=now;
          boundaryGeneration=generation;
          return false;
        }
        return true;
      }
      if(off>rest)rest=off;
      if(elapsed(now,since)<rest)return false;
      // Uma imagem capturada durante a parada já serve. Não exige esperar
      // outro quadro depois do fim da pausa mínima.
      if(!requested || generation==boundaryGeneration || elapsed(now,since)<=sampleAgeMs)return false;
      phase=MOVING;
      since=now;
      budget=on;
      rest=off;
      return true;
    }
  };
  inline const char *name(State s) {
    switch (s) {
      case State::STOP:
        return "PARADO";
      case State::FOLLOW:
        return "LINHA";
      case State::CROSS:
        return "CRUZAMENTO";
      case State::PIVOT:
        return "GIRO";
      case State::SEARCH:
        return "MEMORIA";
      default:
        return "FALHA";
    }
  }
  struct Drive {
    float left = 0;
    float right = 0;
    State state = State::STOP;
    Pace pace = Pace::CONTINUOUS;
    uint32_t holdMs = 0;
    float speedScale = 1;
    // não pode desaparecer na normalização dos dois lados
  };
  class Controller {
    State state = State::STOP;
    uint32_t previous = 0;
    uint32_t lastSeen = 0;
    uint32_t phase = 0;
    uint32_t start = 0;
    float oldError = 0;
    float derivative = 0;
    int direction = 1;
    int stable = 0;
    int severe = 0;
    int crossVotes = 0;
    int oppositeVotes = 0;
    uint32_t pivotPauseAt = 0, pivotPauseMs = 0;
    bool first = true;
    bool hasMemory = false;
    uint32_t memoryAt = 0;
    float memoryNear = cfg::CENTRO_CAMERA;
    float memoryHeading = 0;
    float memoryTurn = 0;
    int severeDirection = 0;
    float motionLeft = 0;
    float motionRight = 0;
    uint32_t motionAt = 0;
    bool haveMotion = false;
    bool recoverBack = false;
    float speedScale = 1;
    float rememberedCurve = 0;
    uint32_t curveAt = 0;
    Pace pace = Pace::CAREFUL;
    Drive output(
        float left = 0,
        float right = 0
        ) const {
      Pace currentPace =
        (state == State::STOP || state == State::FAULT)
        ? Pace::CONTINUOUS
        : (state == State::SEARCH)
        ? Pace::RECOVERY
        : (state == State::PIVOT)
        ? Pace::TURN
        : (state == State::CROSS)
        ? Pace::CROSSING
        : pace;
      return {
        limit(left, -1.0f, 1.0f),
          limit(right, -1.0f, 1.0f),
          state,
          currentPace,
          0,
          speedScale
      };
    }
    Drive atravessarCruzamento() {
      // Ignora erro lateral e ramos do X/+.
      // Durante o cruzamento, os dois lados avancam juntos.
      first = true;
      derivative = 0;
      severe = 0;
      float forward = limit(cfg::BASE_RETA, 0.0f, 1.0f);
      return {
        forward,
          forward,
          state,
          Pace::CROSSING
      };
    }
    public:
    State status() const {
      return state;
    }
    void stop() {
      state = State::STOP;
    }
    void fault() {
      state = State::FAULT;
    }
    void begin(uint32_t now) {
      *this = Controller();
      state = State::FOLLOW;
      previous = now;
      lastSeen = now;
      start = now;
    }
    Drive step(
        const Vision &v,
        uint32_t now
        ) {
      // =========================================================
      // ESTADOS QUE NAO DEVEM MOVER
      // =========================================================
      if (state == State::STOP || state == State::FAULT) {
        return output();
      }
      if (elapsed(now, start) >= cfg::TEMPO_MAX_PROVA_MS) {
        fault();
        return output();
      }
      // =========================================================
      // TEMPO ENTRE QUADROS
      // =========================================================
      float dt = limit(
          elapsed(now, previous) / 1000.0f,
          0.015f,
          0.25f
          );
      previous = now;
      float risk=fmaxf(v.curve,limit(fabsf(v.error),0,1));
      if(v.valid && !v.junction && risk>0.25f){
        rememberedCurve=risk;
        curveAt=now;
      }
      else if(v.junction || elapsed(now,curveAt)>cfg::MEMORIA_CURVA_MS)rememberedCurve=0;
      risk=fmaxf(risk,rememberedCurve);
      speedScale=1-cfg::REDUCAO_ANTECIPACAO*risk;
      // Diminui a agressividade quando a pista esta dificil.
      pace =
        (
         v.cornerAhead ||
         v.cornerNear ||
         risk > 0.25f ||
         fabsf(v.heading) > 0.25f ||
         fabsf(v.error) > 0.22f ||
         v.confidence < 0.8f ||
         dt > 0.12f
        )
        ? Pace::CAREFUL
        : Pace::STRAIGHT;
      // =========================================================
      // PERDEU A FITA
      // =========================================================
      if (!v.valid) {
        stable = 0;
        severe = 0;
        crossVotes = 0;
        oppositeVotes = 0;
        // Tolera uma falha bem curta de leitura.
        if (elapsed(now, lastSeen) < 120) {
          return output();
        }
        if (state != State::SEARCH) {
          recoverBack =
            cfg::RETORNO_ATIVO &&
            state == State::FOLLOW &&
            haveMotion &&
            elapsed(now, motionAt) < 350;
          state = State::SEARCH;
          phase = lastSeen;
        }
      }
      // =========================================================
      // BUSCA / REENCONTRO DA FITA
      // =========================================================
      if (state == State::SEARCH) {
        if (
            !hasMemory ||
            elapsed(now, memoryAt) >= cfg::MEMORIA_FITA_MS
           ) {
          fault();
          return output();
        }
        // Encontrou a fita novamente com informacao suficiente:
        // volta a seguir imediatamente.
        if (v.valid && v.nearCount >= 2) {
          state = State::FOLLOW;
          first = true;
          stable = 0;
          lastSeen = now;
          recoverBack = false;
        }
        else if (v.valid) {
          // Leitura parcial: continua se aproximando devagar.
          const float effort = 0.35f;
          int dir = v.hint;
          if (dir == 0) {
            dir = (memoryTurn >= 0) ? 1 : -1;
          }
          return (dir >= 0)
            ? output(effort, effort * 0.30f)
            : output(effort * 0.30f, effort);
        }
        else {
          uint32_t lost = elapsed(now, phase);
          // Se acabou de perder a fita, desfaz apenas o ultimo avanco.
          if (
              recoverBack &&
              lost < 120 + cfg::RETORNO_MS
             ) {
            Drive back = output(
                -motionLeft,
                -motionRight
                );
            back.holdMs =
              120 +
              cfg::RETORNO_MS -
              lost;
            return back;
          }
          if (
              recoverBack &&
              lost <
              120 +
              cfg::RETORNO_MS +
              cfg::RETORNO_PAUSA_MS
             ) {
            return output();
          }
          const float effort =
            limit(
                cfg::ESFORCO_REENCONTRO,
                0.0f,
                1.0f
                );
          // Quanto mais forte era a curva antes de perder a fita, mais o
          // reencontro fecha o raio. Em curva forte a lateral interna pode
          // parar; a partida posterior recebe impulso de torque no MotorRamp.
          const float bend = limit(
              fabsf(memoryTurn) * 1.8f,
              0.0f,
              1.0f
          );

          const float inner =
              effort * (1.0f - 0.90f * bend);
          return (memoryTurn >= 0)
            ? output(effort, inner)
            : output(inner, effort);
        }
      }
      if (!v.valid) {
        return output();
      }
      lastSeen = now;
      // =========================================================
      // MEMORIA DA ULTIMA POSICAO DA FITA
      // =========================================================
      if (
          v.nearCount >= 2 &&
          (
           v.confidence >= 0.35f ||
           v.cornerNear
          )
         ) {
        hasMemory = true;
        memoryAt = now;
        memoryNear = v.nearX;
        memoryHeading = v.heading;
        // Em cruzamento, nao memoriza ramo lateral.
        memoryTurn =
          (v.junction || state == State::CROSS)
          ? 0.0f
          : v.cornerAhead
          ? static_cast<float>(v.cornerDirection)
          : limit(v.error, -1.0f, 1.0f);
      }
      // =========================================================
      // CRUZAMENTO
      // =========================================================
      if (!v.junction) {
        crossVotes = 0;
      }
      else if (crossVotes < 3) {
        ++crossVotes;
      }
      // So confirma cruzamento depois de 3 quadros.
      if(v.junction)oppositeVotes=0;
      if (
          (state == State::FOLLOW || state == State::PIVOT) &&
          crossVotes >= 3
         ) {
        state = State::CROSS;
        phase = now;
        stable = 0;
      }
      if (state == State::CROSS) {
        bool nearWide = false;
        for (int i = 0; i < cfg::N && cfg::Y[i]>=88; ++i) {
          nearWide |=
            v.rows[i].found &&
            v.rows[i].wide;
        }
        if (
            !v.junction &&
            v.nearCount >= 2 &&
            !nearWide
           ) {
          ++stable;
        }
        else {
          stable = 0;
        }
        if (
            elapsed(now, phase) >
            cfg::CRUZAMENTO_MAX_MS
           ) {
          fault();
          return output();
        }
        if (stable < 3) {
          return atravessarCruzamento();
        }
        state = State::FOLLOW;
        stable = 0;
        first = true;
        derivative = 0;
        severe = 0;
      }
      // Enquanto o cruzamento ainda esta sendo confirmado,
      // segue lentamente para frente em vez de parar.
      if (v.junction) {
        first = true;
        derivative = 0;
        return output(
            cfg::BASE_CURVA,
            cfg::BASE_CURVA
            );
      }
      // =========================================================
      // DIRECAO PROVAVEL DA CURVA
      // =========================================================
      if (state != State::PIVOT) {
        if (v.cornerNear) {
          direction = v.cornerDirection;
        }
        else if (fabsf(v.error) > 0.12f) {
          direction = sign(v.error);
        }
      }
      // =========================================================
      // CURVA FECHADA / PIVOT
      // =========================================================
      if (state == State::PIVOT) {
        if (
            elapsed(now, phase) >
            cfg::GIRO_MAX_MS
           ) {
          fault();
          return output();
        }
        if(elapsed(now,pivotPauseAt)<pivotPauseMs)return output();
        // A segunda curva do S pode chegar antes de aparecer uma reta.
        // Só considera cotovelo próximo ou desvio próximo concordando com o erro.
        float nearOffset=v.nearX-cfg::CENTRO_CAMERA;
        int nextDirection=v.cornerNear?v.cornerDirection:
          ((fabsf(nearOffset)>20 && fabsf(v.error)>0.35f && sign(nearOffset)==sign(v.error))?sign(nearOffset):0);
        if(cfg::MODO_CURVAS_EM_S && v.nearCount>=2 && nextDirection==-direction){
          ++oppositeVotes;
          if(oppositeVotes>=cfg::S_QUADROS_TROCA){
            direction=nextDirection;
            oppositeVotes=0;
            stable=0;
            pivotPauseAt=now;
            pivotPauseMs=cfg::S_PAUSA_TROCA_MS;
            first=true;
            derivative=0;
          }
          return output();
          // já para ao ver a próxima curva; confirma antes de trocar
        }
        oppositeVotes=0;
        bool centered =
          !v.cornerNear &&
          fabsf(v.error) < 0.20f &&
          fabsf(v.heading) < 0.30f;
        bool passed =
          !v.cornerNear &&
          v.error * direction < -0.15f &&
          v.nearCount >= 2 &&
          fabsf(
              v.nearX -
              cfg::CENTRO_CAMERA
              ) < 50;
        if (centered || passed) {
          ++stable;
        }
        else {
          stable = 0;
        }
        if (stable >= 3) {
          state = State::FOLLOW;
          first = true;
          derivative = 0;
          stable = 0;
          severe = 0;
        }
        else if (stable > 0) {
          // Nao continua girando enquanto confirma que centralizou.
          return output();
        }
        else {
          float effort =
            limit(
                cfg::ESFORCO_CURVA_FECHADA,
                0.0f,
                1.0f
                );
          if (
              !v.cornerNear &&
              fabsf(v.error) < 0.40f
             ) {
            effort *= 0.75f;
          }
          // Curva fechada precisa de raio pequeno. A roda interna pode parar
          // temporariamente; ao voltar a mover, MotorRamp aplica impulso de
          // partida para evitar o antigo problema de apenas zumbir.
          const float interna =
              effort * cfg::FRACAO_INTERNA_CURVA_FECHADA;
          return (direction > 0)
            ? output(effort, interna)
            : output(interna, effort);
        }
      }
      // =========================================================
      // CONTROLE PD NORMAL
      // =========================================================
      if (first) {
        oldError = v.error;
        derivative = 0;
        first = false;
      }
      float raw =
        (v.error - oldError) /
        dt;
      oldError = v.error;
      derivative +=
        dt /
        (0.10f + dt) *
        (
         limit(raw, -5.0f, 5.0f) -
         derivative
        );
      float turn =
        limit(
            cfg::KP * v.error +
            cfg::KD * derivative,
            -1.0f,
            1.0f
            );
      if (
          fabsf(v.error) <
          cfg::ZONA_MORTA_ERRO &&
          fabsf(v.heading) < 0.10f
         ) {
        turn = 0;
        derivative = 0;
      }
      // =========================================================
      // DETECCAO DE CURVA MUITO FORTE
      // =========================================================
      const bool hard =
          v.cornerNear ||
          fabsf(v.error) > 0.48f ||
          fabsf(v.nearX - cfg::CENTRO_CAMERA) > 38;
      int candidate =
        v.cornerNear
        ? v.cornerDirection
        : sign(v.error);
      if (hard && candidate != 0) {
        severe =
          (severeDirection == candidate)
          ? severe + 1
          : 1;
        severeDirection = candidate;
      }
      else {
        severe = 0;
        severeDirection = 0;
      }
      // Nao para no primeiro quadro da curva.
      // So entra no modo de curva fechada apos confirmacao.
      const int severeRequired = v.cornerNear ? 1 : 2;

      if (severe >= severeRequired) {
        direction = candidate;
        state = State::PIVOT;
        phase = now;
        stable = 0;
        oppositeVotes=0;
        pivotPauseAt=now;
        pivotPauseMs=cfg::ACOMODAR_GIRO_MS;
        return output();
        // corta avanço por ACOMODAR_GIRO_MS; depois gira
      }
      // =========================================================
      // CURVA NORMAL
      // =========================================================
      float bend =
        limit(
            fabsf(turn),
            0.0f,
            1.0f
            );
      float base =
        cfg::BASE_RETA +
        (
         cfg::BASE_CURVA -
         cfg::BASE_RETA
        ) *
        bend;
      if (
          v.confidence < 0.6f ||
          state == State::CROSS
         ) {
        base = cfg::BASE_CURVA;
      }
      float differential =
        base *
        limit(
            turn * 0.65f,
            -0.85f,
            0.85f
            );
      Drive drive;
      // Em uma correcao forte, nao zera a roda interna. O lado de dentro
      // permanece com parte do esforco do lado externo para manter torque.
      if (fabsf(turn) >= cfg::LIMIAR_CURVA_FORTE) {
        const float outer = base;
        const float inner = base * cfg::FRACAO_INTERNA_CURVA_FORTE;
        drive = (turn > 0)
          ? output(outer, inner)
          : output(inner, outer);
      }
      else {
        drive =
          output(
              limit(
                base + differential,
                0.0f,
                1.0f
                ),
              limit(
                base - differential,
                0.0f,
                1.0f
                )
              );
      }
      motionLeft = drive.left;
      motionRight = drive.right;
      motionAt = now;
      haveMotion = true;
      return drive;
    }
  };
  // Mapeia esforco em PWM com piso; zero continua sendo parada verdadeira.
  inline int pwmFor(float effort,int minimum) {
    if(fabsf(effort)<0.035f)return 0;
    return sign(effort)*(int)(minimum+(cfg::PWM_MAX-minimum)*limit(fabsf(effort),0,1)+0.5f);
  }
  // PWM direto no seguimento: piso mecanico nao pode anular a correcao visual.
  struct PwmDrive {
    int left=0,right=0;
  };
  // Impulso proporcional: elevar ambos os lados ao mesmo mínimo anularia a
  // direção durante todo um passo curto. Zero permanece parado.
  inline PwmDrive startupForDrive(const PwmDrive &d){
    int peak=(int)fmaxf(fabsf((float)d.left),fabsf((float)d.right));
    if(peak==0)return {};
    int boost=peak>cfg::PWM_PARTIDA?peak:cfg::PWM_PARTIDA;
    return {(int)(fabsf((float)d.left)*boost/peak+0.5f),
      (int)(fabsf((float)d.right)*boost/peak+0.5f)};
  }
  inline PwmDrive pwmForDrive(const Drive &d) {
    if (d.state == State::STOP || d.state == State::FAULT) return {};
    const float peak = fmaxf(fabsf(d.left), fabsf(d.right));
    if (peak < 0.001f) return {};
    if (d.state == State::FOLLOW ||
        d.state == State::CROSS ||
        d.state == State::SEARCH) {
      int ceiling = cfg::PWM_RETA;
      const bool steering =
        d.state == State::FOLLOW &&
        d.left >= 0 &&
        d.right >= 0 &&
        fabsf(d.left - d.right) > peak * 0.25f;
      if (d.state == State::CROSS) {
        ceiling = cfg::PWM_CRUZAMENTO;
      }
      else if (d.state == State::SEARCH) {
        // O reencontro tem PWM proprio; nao fica preso ao PWM_CAUTELA.
        ceiling = cfg::PWM_REENCONTRO;
      }
      else if (steering) {
        ceiling = cfg::PWM_CURVA;
      }
      else if (d.pace != Pace::STRAIGHT) {
        ceiling = cfg::PWM_CAUTELA;
      }
      if (d.state == State::FOLLOW) {
        ceiling =
          (int)(ceiling * limit(d.speedScale, 0.0f, 1.0f) + 0.5f);
      }
      const int floorLeft = cfg::PWM_MIN_ESQ;
      const int floorRight = cfg::PWM_MIN_DIR;
      if (d.state == State::FOLLOW) {
        const int floor =
          (int)fmaxf((float)floorLeft, (float)floorRight);
        if (ceiling < floor) ceiling = floor;
      }
      int left =
        (int)(fabsf(d.left) / peak * ceiling + 0.5f) * sign(d.left);
      int right =
        (int)(fabsf(d.right) / peak * ceiling + 0.5f) * sign(d.right);
      // IMPORTANTE: durante FOLLOW nao force cada roda ao PWM_MIN.
      // Fazer isso achata a diferenca entre os lados e faz o carrinho seguir
      // quase reto justamente quando deveria fechar a curva. O MotorRamp ja
      // fornece o impulso inicial quando uma roda volta a sair do zero.
      if (d.state == State::SEARCH || d.state == State::CROSS) {
        if (left > 0 && left < floorLeft) left = floorLeft;
        if (left < 0 && -left < floorLeft) left = -floorLeft;
        if (right > 0 && right < floorRight) right = floorRight;
        if (right < 0 && -right < floorRight) right = -floorRight;
      }

      return {left, right};
    }
    int left = pwmFor(d.left, cfg::PWM_MIN_ESQ);
    int right = pwmFor(d.right, cfg::PWM_MIN_DIR);
    int maxPwm =
      (int)fmaxf(fabsf((float)left), fabsf((float)right));
    if (maxPwm > cfg::PWM_GIRO) {
      left = left * cfg::PWM_GIRO / maxPwm;
      right = right * cfg::PWM_GIRO / maxPwm;
    }
    return {left, right};
  }
  inline bool commandWindowOpen(uint32_t age,uint32_t holdMs){
    return holdMs==0 || age<holdMs;
  }
  static_assert(cfg::ROI_TOPO_Y>=0 && cfg::ROI_TOPO_Y<=64,"ROI deve manter faixas proximas e intermediarias");
  static_assert(cfg::PWM_CAUTELA > 0 && cfg::PWM_CAUTELA <= cfg::PWM_MAX, "PWM_CAUTELA invalido");
  static_assert(cfg::PWM_REENCONTRO > 0 && cfg::PWM_REENCONTRO <= cfg::PWM_MAX, "PWM_REENCONTRO invalido");
  static_assert(cfg::PWM_CURVA > 0 && cfg::PWM_CURVA <= cfg::PWM_MAX, "PWM_CURVA invalido");
  static_assert(cfg::PWM_CRUZAMENTO > 0 && cfg::PWM_CRUZAMENTO <= cfg::PWM_MAX, "PWM_CRUZAMENTO invalido");
  static_assert(cfg::PWM_GIRO > 0 && cfg::PWM_GIRO <= cfg::PWM_MAX, "PWM_GIRO invalido");
  static_assert(cfg::PWM_RETA > 0 && cfg::PWM_RETA <= cfg::PWM_MAX, "PWM_RETA invalido");
  static_assert(cfg::FRACAO_INTERNA_CURVA_FORTE > 0.0f && cfg::FRACAO_INTERNA_CURVA_FORTE <= 1.0f,
      "FRACAO_INTERNA_CURVA_FORTE invalida");
  static_assert(cfg::FRACAO_INTERNA_CURVA_FECHADA >= 0.0f && cfg::FRACAO_INTERNA_CURVA_FECHADA <= 1.0f,
      "FRACAO_INTERNA_CURVA_FECHADA invalida");
  static_assert(cfg::S_QUADROS_TROCA>=2,"Troca do S exige pelo menos dois quadros");
  static_assert(cfg::S_PAUSA_TROCA_MS>=cfg::REVERSAO_MS && cfg::S_PAUSA_TROCA_MS<cfg::GIRO_MAX_MS,
      "Pausa da troca de curva invalida");
  static_assert(cfg::REDUCAO_ANTECIPACAO>=0 && cfg::REDUCAO_ANTECIPACAO<1,"Reducao deve ser de 0 a menos de 1");
  static_assert(cfg::COTOVELO_PERTO_Y>=cfg::ROI_TOPO_Y && cfg::COTOVELO_PERTO_Y<=104,"Posicao do giro fora da area observada");
  static_assert(cfg::PWM_MIN_ESQ>0 && cfg::PWM_MIN_ESQ<=cfg::PWM_CAUTELA &&
      cfg::PWM_MIN_DIR>0 && cfg::PWM_MIN_DIR<=cfg::PWM_CAUTELA,"Piso mecanico deve caber no PWM de cautela");
  // Estado de uma roda/lado: inversao passa por zero por REVERSAO_MS.
  class MotorRamp {
    int current=0,lastSign=0,lastTarget=0;
    uint32_t zeroAt=0,kickAt=0;
    bool kicking=false;
    public:
    int value()const{return current;
    }
    int tick(int target,uint32_t now,bool emergency=false,int startupPwm=cfg::PWM_PARTIDA) {
      int desired=sign((float)target);
      if(emergency||desired==0){
        if(current!=0)zeroAt=now;
        current=0;
        kicking=false;
        lastTarget=0;
        return 0;
      }
      if(current && desired!=sign((float)current)){
        current=0;
        zeroAt=now;
        kicking=false;
        return 0;
      }
      if(current==0){
        if(lastSign && desired!=lastSign && elapsed(now,zeroAt)<cfg::REVERSAO_MS)return 0;
        kickAt=now;
        kicking=true;
        lastSign=desired;
      }
      int magnitude=(int)fabsf((float)target);
      // Uma nova ordem de reduzir tem prioridade sobre o impulso de partida.
      if(lastTarget>0 && magnitude<lastTarget)kicking=false;
      lastTarget=magnitude;
      if(kicking && elapsed(now,kickAt)<cfg::IMPULSO_MS){
        if(magnitude<startupPwm)magnitude=startupPwm;
      }
      else kicking=false;
      // Nao arrasta a partida pela faixa de PWM incapaz de mover o motor.
      if(current==0)current=desired*magnitude;
      else if(magnitude<(int)fabsf((float)current))current=desired*magnitude;
      else {
        int delta=desired*magnitude-current;
        if (delta > 7) delta = 7;
        if (delta < -7) delta = -7;
        current+=delta;
      }
      return current;
    }
  };
}
