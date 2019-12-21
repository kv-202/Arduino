
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
  static const uint8_t BANK_MASK = 0xC0;
  static const uint8_t BANK_REG_MASK = 0x03;
  static const uint8_t SPRD_MASK = 0x80;

  static const uint8_t EIE = 0x1B;
  static const uint8_t EIR = 0x1C;
  static const uint8_t ESTAT = 0x1D;
  static const uint8_t ECON2 = 0x1E;
  static const uint8_t ECON1 = 0x1F;

  static const uint8_t ERDPT = 0x00;
  static const uint8_t EWRPT = 0X02;

  static const uint8_t BANK2 = 0x80;
  static const uint8_t MICON = 0x11 | BANK2;
  static const uint8_t MICMD = 0x12 | BANK2;
  static const uint8_t MIREGADR = 0x14 | BANK2;
  static const uint8_t MIW = 0x16 | BANK2;
  static const uint8_t MIR = 0x18 | BANK2;

  static const uint8_t BANK3 = 0xC0;
  static const uint8_t MISTAT = 0x0A | BANK3;

  static const uint8_t MICMD_MIIRD = 0x01;
  static const uint8_t MISTAT_BUSY = 0x01;
  
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
    set_bank((reg & BANK_MASK) >> 6);
    tr_start();
    tr(SET_BITS | (ADDR_MASK & reg));
    tr(value);
    tr_stop();
  }

  void reset_bits(uint8_t reg, uint8_t value) {
    set_bank((reg & BANK_MASK) >> 6);
    tr_start();
    tr(RESET_BITS | (ADDR_MASK & reg));
    tr(value);
    tr_stop();
  }
 
  void write(uint8_t reg, uint8_t value) {
    set_bank((reg & BANK_MASK) >> 6);
    tr_start();
    tr(WR | (ADDR_MASK & reg));
    tr(value);
    tr_stop();
  }

  void write16(uint8_t reg, uint16_t value) {
    write(reg, (uint8_t)(0xFF & value));
    write(reg + 1, (uint8_t)(0xFF & (value >> 8)));
  }

  void miwrite(uint8_t reg, uint16_t value) {
    write(MIREGADR, reg);
    write16(MIW, value);
    while (read(MISTAT) & MISTAT_BUSY) {};
  }

  void write_buff(uint16_t addr, uint8_t *buff, uint16_t len) {
    write16(EWRPT, addr);
    tr_start();
    tr(WB);
    for (uint16_t i = 0; i < len; i++) {
      tr(buff[i]);
    }
    tr_stop();
  }

  uint8_t read(uint8_t reg) {
    set_bank((reg & BANK_MASK) >> 6);
    tr_start();
    tr(RR | (ADDR_MASK & reg));
    if (reg & 0x80) {
      tr(0);
    }
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

  uint16_t miread(uint8_t reg) {
    write(MIREGADR, reg);
    write(MICMD, MICMD_MIIRD);
    while (read(MISTAT) & MISTAT_BUSY) {};
    write(MICMD, 0x00);
    uint8_t retl = read(MIR);
    return (((uint16_t)read(MIR+1))<<8) | retl; 
  }

  void read_buff(uint16_t addr, uint8_t *buff, uint16_t len) {
    write16(ERDPT, addr);
    tr_start();
    tr(RB);    
    for (uint16_t i = 0; i < len; i++) {
      buff[i] = tr(0);
    }
    tr_stop();
  }
};

struct mac_addr {
  uint8_t _addr[6];

  mac_addr() {
    _addr[0] = _addr[1] = _addr[2] = _addr[3] = _addr[4] = _addr[5] = 0; 
  }

  mac_addr(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5) {
    _addr[0] = p0;
    _addr[1] = p1;
    _addr[2] = p2;
    _addr[3] = p3;
    _addr[4] = p4;
    _addr[5] = p5; 
  }

  void print(){
    Serial.print(_addr[0], HEX);
    Serial.print(":");
    Serial.print(_addr[1], HEX);
    Serial.print(":");
    Serial.print(_addr[2], HEX);
    Serial.print(":");
    Serial.print(_addr[3], HEX);
    Serial.print(":");
    Serial.print(_addr[4], HEX);
    Serial.print(":");
    Serial.print(_addr[5], HEX);  
  }

  void println(){
    print();
    Serial.println();  
  }
};

struct ip_addr {
  uint8_t _addr[4];

  ip_addr() {
    _addr[0] = _addr[1] = _addr[2] = _addr[3] = 0;
  }

  ip_addr(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3) {
    _addr[0] = p0;
    _addr[1] = p1;
    _addr[2] = p2;
    _addr[3] = p3;
  }

  void print() {
    Serial.print(_addr[0]);
    Serial.print(".");
    Serial.print(_addr[1]);
    Serial.print(".");
    Serial.print(_addr[2]);
    Serial.print(".");
    Serial.print(_addr[3]);
  }

  void println() {
    print();
    Serial.println();
  }
};

struct enc28j60_header {
  uint16_t _next_pkg_ptr;
  uint16_t _frame_len;
  uint16_t _status;

  bool is_valid() { return _status & 0x80; }

  void print() {
    Serial.print("next page ptr: ");
    Serial.println(_next_pkg_ptr, HEX);
    Serial.print("len: ");
    Serial.println(_frame_len);
    Serial.print("status: ");
    Serial.println(_status, HEX);
  }
};

struct ethernet_header {
  mac_addr _dest_addr;
  mac_addr _source_addr;
  uint16_t _type;

  void print() {
    Serial.print("dest addr: ");
    _dest_addr.println();
    Serial.print("source addr: ");
    _source_addr.println();
    Serial.print("type: ");
    Serial.println(_type, HEX);
  }
};

struct arp_pkg {
  uint16_t _htype;
  uint16_t _ptype;
  uint8_t _hlen;
  uint8_t _plen;
  uint16_t _opcode;
  mac_addr _sha;
  ip_addr _spa;
  mac_addr _tha;
  ip_addr _tpa;

  void print() {
    Serial.print("htype: 0x");
    Serial.println(_htype, HEX);
    Serial.print("ptype: 0x");
    Serial.println(_ptype, HEX);
    Serial.print("hlen: ");
    Serial.println(_hlen);
    Serial.print("plen: ");
    Serial.println(_plen);
    Serial.print("opcode: 0x");
    Serial.println(_opcode, HEX);
    Serial.print("sha: ");
    _sha.println();
    Serial.print("spa: ");
    _spa.println();
    Serial.print("tha: ");
    _tha.println();
    Serial.print("tpa: ");
    _tpa.println();    
  }
};

class enc28j60 {

  static const uint8_t ERBSTART = 0x08;
  static const uint8_t ERBEND = 0x0A;
  static const uint8_t ERXRDPT = 0x0C;

  static const uint8_t ECON1 = 0x1F;
  static const uint8_t ECON2 = 0x1E;

  static const uint8_t ECON1_TXRST = 0x80;
  static const uint8_t ECON1_RXEN = 0x04;

  static const uint8_t ECON2_PKTDEC = 0x40;

  static const uint8_t BANK1 = 0x40;
  static const uint8_t EPKTCNT = 0x19 | BANK1;

  static const uint8_t BANK2 = 0x80;
  static const uint8_t MACON1 = 0x00 | BANK2;
  static const uint8_t MACON2 = 0x01 | BANK2;
  static const uint8_t MACON3 = 0x02 | BANK2;
  static const uint8_t MACON4 = 0x03 | BANK2;
  static const uint8_t MABBIPG = 0x04 | BANK2;
  static const uint8_t MAIPG = 0x06 | BANK2;
  static const uint8_t MAMXFL = 0x0A | BANK2;

  static const uint8_t MACON1_INIT = 0x0D;
  static const uint8_t MACON3_INIT = 0x30;

  static const uint16_t FRAMESIZE = 1500;

  static const uint8_t BANK3 = 0xC0;
  static const uint8_t MAADR1 = 0x00 | BANK3;
  static const uint8_t MAADR0 = 0x01 | BANK3;
  static const uint8_t MAADR3 = 0x02 | BANK3;
  static const uint8_t MAADR2 = 0x03 | BANK3;
  static const uint8_t MAADR5 = 0x04 | BANK3;
  static const uint8_t MAADR4 = 0x05 | BANK3;

  static const uint16_t RXSIZE = 0x1A00;

  static const uint8_t PHCON1 = 0x00;
  static const uint8_t PHCON2 = 0x05;
  static const uint8_t PHLCON = 0x14;

  static const uint16_t PHCON1_PDPXMD = 0x0C00;
  static const uint16_t PHCON2_HDLDIS = 0x0100;
  static const uint16_t PHLCON_LAONPUT = 0x0100;
  static const uint16_t PHLCON_LBONPUT = 0x0020;
  static const uint16_t PHLCON_MAX_TIME = 0x000A; 
    
  encspi _spi;
  uint16_t _next_pkg_ptr;
  
public:
  void init(const mac_addr &addr){
    Serial.println("Start init enc28j60 module....");
    _spi.init();

    _spi.write16(ERBSTART, 0);
    _spi.write16(ERBEND, RXSIZE);
    _spi.write16(ERXRDPT, 0);
    _next_pkg_ptr = 0;

    _spi.write(MACON2, 0);
    _spi.set_bits(MACON1, MACON1_INIT);
    _spi.set_bits(MACON3, MACON3_INIT);
    _spi.write16(MAMXFL, FRAMESIZE);
    _spi.write(MABBIPG, 0x15);
    _spi.write16(MAIPG, 0x0c12);
    set_mac_addr(addr);

    mac_addr raddr = get_mac_addr();
    Serial.print("mac: ");
    raddr.println();

//    _spi.miwrite(PHCON1, PHCON1_PDPXMD);
    _spi.miwrite(PHCON2, PHCON2_HDLDIS);
    _spi.miwrite(PHLCON, PHLCON_LAONPUT | PHLCON_LBONPUT | PHLCON_MAX_TIME);
    _spi.set_bits(ECON1, ECON1_RXEN);
    Serial.println();
  }

  void set_mac_addr(const mac_addr &addr) {
    _spi.write(MAADR0, addr._addr[0]);
    _spi.write(MAADR1, addr._addr[1]);
    _spi.write(MAADR2, addr._addr[2]);
    _spi.write(MAADR3, addr._addr[3]);
    _spi.write(MAADR4, addr._addr[4]);
    _spi.write(MAADR5, addr._addr[5]);
  }

  mac_addr get_mac_addr() {
    mac_addr ret;
    ret._addr[0] = _spi.read(MAADR0);
    ret._addr[1] = _spi.read(MAADR1);
    ret._addr[2] = _spi.read(MAADR2);
    ret._addr[3] = _spi.read(MAADR3);
    ret._addr[4] = _spi.read(MAADR4);
    ret._addr[5] = _spi.read(MAADR5);
    return ret;
  }

  void read_pkg_to_prot(uint16_t type, uint16_t buff_pos, uint16_t len) {
    switch (type){
      case 0x0608: {
        if (sizeof(arp_pkg) <= len) {
          arp_pkg apr;
          _spi.read_buff(buff_pos, (uint8_t*)&apr, sizeof(arp_pkg));
          apr.print();
        }
        else
        {
          Serial.println("arp so small.");
        }
      }
      break;
    }
  }

  void read_pkg() {
    if (_spi.read(EPKTCNT) > 0) {
      enc28j60_header enc28j60_h;
      _spi.read_buff(_next_pkg_ptr, (uint8_t*)&enc28j60_h, sizeof(enc28j60_header));
      if (enc28j60_h.is_valid()) {
        enc28j60_h.print();
        if (enc28j60_h._frame_len >= sizeof(ethernet_header)) {
          uint16_t buff_pos = _next_pkg_ptr + sizeof(enc28j60_header);
          ethernet_header ethernet_h;
          _spi.read_buff(buff_pos, (uint8_t*)&ethernet_h, sizeof(ethernet_header));
          ethernet_h.print();
          buff_pos += sizeof(ethernet_header);
          read_pkg_to_prot(ethernet_h._type, buff_pos, enc28j60_h._frame_len - sizeof(ethernet_header));
        } 
      }
      else
      {
        Serial.println("Wrong pkg !!!");
      }
      Serial.println();
      _next_pkg_ptr = enc28j60_h._next_pkg_ptr;
      _spi.write16(ERXRDPT, _next_pkg_ptr - 1);
      _spi.set_bits(ECON2, ECON2_PKTDEC);
    }
  }

  void write_pkg() {
    while(_spi.read(ECON1) & ECON1_TXRST) {};
  }
};

enc28j60 enc;

void setup() {
  Serial.begin(9600);
  enc.init(mac_addr(0x74,0x69,0x69,0x2D,0x30,0x31));
}

void loop() {
  enc.read_pkg();
  
  // put your main code here, to run repeatedly:
}
