
class encspi {
  static const int DATAOUT = 12;
  static const int DATAIN = 11;
  static const int CLOCK = 13;
  static const int SLAVESELECT = 10;

  static const uint8_t RR = 0;
  static const uint8_t RB = 0x3A;
  static const uint8_t WR = 0x40;
  static const uint8_t WB = 0x7A;
  static const uint8_t SET_BITS = 0x80;
  static const uint8_t RESET_BITS = 0xA0;
  static const uint8_t RESET = 0xFF;

  static const uint8_t ADDR_MASK = 0x1F;
  static const uint8_t BANK_MASK = 0x60;
  static const uint8_t BANK_REG_MASK = 0x03;
  static const uint8_t SPRD_MASK = 0x80;

  static const uint8_t EIE = 0x1B;
  static const uint8_t EIR = 0x1C;
  static const uint8_t ESTAT = 0x1D;
  static const uint8_t ECON2 = 0x1E;
  static const uint8_t ECON1 = 0x1F;

  int8_t _bank;
  
public:
  encspi():
    _bank(-1)
  {}

  void init() {  
    pinMode(SLAVESELECT, OUTPUT);
    digitalWrite(SLAVESELECT, HIGH);

    pinMode(DATAIN, OUTPUT);
    pinMode(DATAOUT, INPUT);
    pinMode(CLOCK, OUTPUT);

    digitalWrite(DATAIN, HIGH);
    digitalWrite(DATAIN, LOW);
    digitalWrite(CLOCK, LOW);

    SPCR = bit(SPE) | bit(MSTR);
    bitSet(SPSR, SPI2X);
    tr_stop();

    tr_start();
    tr(RESET);
    tr_stop();

    delay(10);
  }

  static uint8_t tr(uint8_t data) {  
    SPDR = data;
    while (!(SPSR & (1 << SPIF))) {};    
    return SPDR;
  }

  static void tr_start() {
    cli();
    digitalWrite(SLAVESELECT, LOW);
  }

  static void tr_stop() {
    digitalWrite(SLAVESELECT, HIGH);
    sei();
  }

  void set_bank(uint8_t bank) {
    if (_bank == (int8_t)bank) {
      return;
    }
    
    tr_start();
    tr(RESET_BITS | ECON1);
    tr(BANK_REG_MASK);
    tr_stop();
    tr_start();
    tr(SET_BITS | ECON1);
    tr(bank & BANK_REG_MASK);
    tr_stop();
    _bank = (int8_t)bank;
  }

  void set_bits(uint8_t reg, uint8_t value) {
    set_bank((reg & BANK_MASK) >> 5);
    tr_start();
    tr(SET_BITS | (ADDR_MASK & reg));
    tr(value);
    tr_stop();
  }
 
  void write(uint8_t reg, uint8_t value) {
    set_bank((reg & BANK_MASK) >> 5);
    tr_start();
    tr(WR | (ADDR_MASK & reg));
    tr(value);
    tr_stop();
  }

  void write16(uint8_t reg, uint16_t value) {
    write(reg, (uint8_t)(0xFF & value));
    write(reg + 1, (uint8_t)(0xFF & (value >> 8)));
  }

  uint8_t read(uint8_t reg) {
    set_bank((reg & BANK_MASK) >> 5);
    tr_start();
    tr(RR | (ADDR_MASK & reg));
    uint8_t ret = tr(0);
    tr_stop();
    return ret;
  }

  uint16_t read16(uint8_t reg) {
    uint16_t ret = 0;
    ret = ((uint16_t)read(reg + 1)) << 8;
    ret |= (uint16_t)read(reg);
    return ret;
  }
};

struct mac_addr {
  uint8_t _p0, _p1, _p2, _p3, _p4, _p5;

  mac_addr():
    _p0(0), _p1(0), _p2(0), _p3(0), _p4(0), _p5(0)
  {}

  mac_addr(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5):
    _p0(p0), _p1(p1), _p2(p2), _p3(p3), _p4(p4), _p5(p5)
  {}
};

class enc28j60 {

  static const uint8_t ERBSTART = 0x08;
  static const uint8_t ERBEND = 0x0A;
  static const uint8_t ERBPTR = 0x0C;

  static const uint8_t BANK2 = 0x80;
  static const uint8_t MACON1 = 0x00 | BANK2;
  static const uint8_t MACON2 = 0x01 | BANK2;
  static const uint8_t MACON3 = 0x02 | BANK2;
  static const uint8_t MACON4 = 0x03 | BANK2;

  static const uint8_t MACON1_INIT = 0x0D;
  static const uint8_t MACON3_INIT = 0x30;

  static const uint8_t BANK3 = 0x90;
  static const uint8_t MAADR1 = 0x00 | BANK3;
  static const uint8_t MAADR0 = 0x01 | BANK3;
  static const uint8_t MAADR3 = 0x02 | BANK3;
  static const uint8_t MAADR2 = 0x03 | BANK3;
  static const uint8_t MAADR5 = 0x04 | BANK3;
  static const uint8_t MAADR4 = 0x05 | BANK3;

  static const uint16_t RXSIZE = 0x1A00;
  
  encspi _spi;
  
public:
  void init(){
    Serial.println("Start init enc28j60 module....");
    _spi.init();

    _spi.write16(ERBSTART, 0);
    _spi.write16(ERBEND, RXSIZE);
    _spi.write16(ERBPTR, 0);

    _spi.write(MACON2, 0);
    _spi.set_bits(MACON1, MACON1_INIT);
    _spi.set_bits(MACON3, MACON3_INIT);

    set_mac_addr(mac_addr(6, 5, 4, 3, 2, 10));
  }

  void set_mac_addr(const mac_addr &addr) {
    _spi.write(MAADR0, addr._p0);
    _spi.write(MAADR1, addr._p1);
    _spi.write(MAADR2, addr._p2);
    _spi.write(MAADR3, addr._p3);
    _spi.write(MAADR4, addr._p4);
    _spi.write(MAADR5, addr._p5);
  }

  mac_addr get_mac_addr() {
    mac_addr ret;
    ret._p0 = _spi.read(MAADR0);
    ret._p1 = _spi.read(MAADR1);
    ret._p2 = _spi.read(MAADR2);
    ret._p3 = _spi.read(MAADR3);
    ret._p4 = _spi.read(MAADR4);
    ret._p5 = _spi.read(MAADR5);
    return ret;
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
