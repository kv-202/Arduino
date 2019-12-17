
class encspi {
  static const int DATAOUT = /*11*/12;
  static const int DATAIN = /*12*/11;
  static const int CLOCK = 13;
  static const int SLAVESELECT = 10;

  static const uint8_t RR = 0;
  static const uint8_t RB = 0x20;
  static const uint8_t WR = 0x40;
  static const uint8_t WB = 0x50;
  static const uint8_t SET_BITS = 0x80;
  static const uint8_t RESET_BITS = 0xA0;

  static const uint8_t BANK_MASK = 0x03;

  static const uint8_t ECON1 = 0x1E;

public:
  void set_bank(uint8_t bank) {
    
    digitalWrite(SLAVESELECT, LOW);
    tr(RESET_BITS | ECON1);
    tr(0x03);
    tr(SET_BITS | ECON1);
    tr(bank);
    digitalWrite(SLAVESELECT, HIGH);
  }

  uint8_t get_bank() {
    
    digitalWrite(SLAVESELECT, LOW);
    tr(RR | ECON1);
    uint8_t ret = tr(0);
    digitalWrite(SLAVESELECT, HIGH);
    return ret & BANK_MASK;
  }
  
public:
  void init() {
    pinMode(DATAOUT, OUTPUT);
    pinMode(DATAIN, INPUT);
    pinMode(CLOCK, OUTPUT);
    pinMode(SLAVESELECT, OUTPUT);
    digitalWrite(SLAVESELECT, HIGH);

    SPCR = (1<<SPE)|(1<<MSTR);

    delay(10);
  }

  static uint8_t tr(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF))) {};
    return SPDR;
  }

  void write(uint8_t reg, uint8_t value) {
  }

  uint8_t read(uint8_t reg) {
    return 0;
  }
};

class enc28j60 {
  encspi spi;
  
public:
  void init(){
    Serial.println("Start init enc28j60 module....");
    spi.init();

    spi.set_bank(1);
    Serial.println(spi.get_bank());
  }
};

enc28j60 enc;

void setup() {
  Serial.begin(9600);
  enc.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}
