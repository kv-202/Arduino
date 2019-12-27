#define htons(a)            ((((a)>>8)&0xff)|(((a)<<8)&0xff00))
#define htonl(a)            ( (((a)>>24)&0xff) | (((a)>>8)&0xff00) | (((a)<<8)&0xff0000) | (((a)<<24)&0xff000000) )


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

  void read_buff_test(uint16_t addr, uint8_t *buff, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
      write16(ERDPT, addr + i);
      tr_start();
      tr(RB);    
      buff[i] = tr(0);
      tr_stop();
    }
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

  bool operator == (const mac_addr &value) {
    for (int i = 0; i < 6; i++) {
      if (_addr[i] != value._addr[i]) {
        return false;
      }
    }
    return true;
  }

  void print() {
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

  void println() {
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

  bool operator == (const ip_addr &value) {
    for (int i = 0; i < 4; i++) {
      if (_addr[i] != value._addr[i]) {
        return false;
      }
    }
    return true;
  }

  bool operator != (const ip_addr &value) {
    return !(*this == value);
  }

  void print() const {
    Serial.print(_addr[0]);
    Serial.print(".");
    Serial.print(_addr[1]);
    Serial.print(".");
    Serial.print(_addr[2]);
    Serial.print(".");
    Serial.print(_addr[3]);
  }

  void println() const {
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
  static const uint16_t TYPE_ARP = 0x0608;
  static const uint16_t TYPE_IP = 0x0008;
  
  mac_addr _to_addr;
  mac_addr _from_addr;
  uint16_t _type;

  void print() {
    Serial.print("to addr: ");
    _to_addr.println();
    Serial.print("from addr: ");
    _from_addr.println();
    Serial.print("type: ");
    Serial.println(_type, HEX);
  }
};

struct arp_pkg {
  static const uint16_t HTYPE_ETHERNET = 0x0100;
  static const uint16_t PTYPE_IPV4 = 0x0008;

  static const uint16_t OPCODE_REQ = 0x0100;
  static const uint16_t OPCODE_RESP = 0x0200;

  static const uint8_t HLEN = 6;
  static const uint8_t PLEN = 4;
  
  uint16_t _htype;
  uint16_t _ptype;
  uint8_t _hlen;
  uint8_t _plen;
  uint16_t _opcode;
  mac_addr _sha;
  ip_addr _spa;
  mac_addr _tha;
  ip_addr _tpa;

  void init() {
    _htype = HTYPE_ETHERNET;
    _ptype = PTYPE_IPV4;
    _hlen = HLEN;
    _plen = PLEN;
  }

  bool is_valid() {
    if (_htype != HTYPE_ETHERNET) {
      Serial.print("Invalid htype value: ");
      Serial.println(_htype, HEX);
      return false;
    }
    if (_ptype != PTYPE_IPV4) {
      Serial.print("Invalid ptype value: ");
      Serial.println(_ptype, HEX);
      return false;
    }
    if (_hlen != HLEN) {
      Serial.print("Invalid hlen value: ");
      Serial.println(_hlen, HEX);
      return false;
    }
    if (_plen != PLEN) {
      Serial.print("Invalid plen value: ");
      Serial.println(_plen, HEX);
      return false;
    }
    if ((_opcode != OPCODE_REQ) && (_opcode != OPCODE_RESP)) {
      Serial.print("Invalid opcode value: ");
      Serial.println(_opcode, HEX);
      return false;
    }
    return true;
  }

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

struct arp_pkg_buff {
  ethernet_header _header;
  arp_pkg _pkg;

  void init() {
    _header._type = ethernet_header::TYPE_ARP;
    _pkg.init();
  }

  void init_req(const ip_addr &to_ip_addr, const mac_addr &from_mac_addr, const ip_addr &from_ip_addr) {
    init();
    _header._to_addr = mac_addr(0xff, 0xff, 0xff, 0xff, 0xff, 0xff);
    _header._from_addr = from_mac_addr;
    _pkg._opcode = arp_pkg::OPCODE_REQ;
    _pkg._sha = from_mac_addr;
    _pkg._spa = from_ip_addr;
    _pkg._tpa = to_ip_addr;
  }

  void init_resp(const mac_addr &to_mac_addr, const ip_addr &to_ip_addr, const mac_addr &from_mac_addr, const ip_addr &from_ip_addr) {
    init();
    _header._to_addr = to_mac_addr;
    _header._from_addr = from_mac_addr;
    _pkg._opcode = arp_pkg::OPCODE_RESP;
    _pkg._sha = from_mac_addr;
    _pkg._spa = from_ip_addr;
    _pkg._tha = to_mac_addr;
    _pkg._tpa = to_ip_addr;
  }

  void print() {
    _header.print();
    _pkg.print();
  }
};

struct ip_pkg {
  static const uint8_t TYPE_ICMP = 0x01;  
  
  static const uint8_t VER_MASK = 0xF0;
  static const uint8_t IHL_MASK = 0x0F;
 
  uint8_t _ver;
  uint8_t _dscp;
  uint16_t _len;
  uint16_t _id;
  uint16_t _offset;
  uint8_t _ttl;
  uint8_t _type;
  uint16_t _check_sum;
  ip_addr _from_addr;
  ip_addr _to_addr;

  static uint16_t check_sum(uint32_t sum, uint8_t *buf, uint16_t len) {
    while(len >= 2) {
        sum += ((uint16_t)*buf << 8) | *(buf+1);
        buf += 2;
        len -= 2;
    }
    if(len) {
        sum += (uint16_t)*buf << 8;
    }
    while(sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return ~htons((uint16_t)sum);
  }

  uint8_t get_ver() const { return (_ver & VER_MASK) >> 4; }

  uint8_t get_ihl() const { return (_ver & IHL_MASK); }

  uint16_t get_len() const { return htons(_len); }

  uint16_t get_id() const { return htons(_id); }

  uint16_t get_calc_check_sum() {
    uint16_t old_check_sum = _check_sum;
    _check_sum = 0;
    uint16_t new_check_sum = check_sum(0, (uint8_t*)this, sizeof(ip_pkg));
    _check_sum = old_check_sum;
    return new_check_sum;
  }

  bool is_check_sum_valid() {
    return get_calc_check_sum() == _check_sum;
  }

  void print() {
    Serial.print("ver: ");
    Serial.println(get_ver());
    Serial.print("ihl: ");
    Serial.println(get_ihl());
    Serial.print("len: ");
    Serial.println(get_len());
    Serial.print("id: 0x");
    Serial.println(get_id(), HEX);
    Serial.print("offset: 0x");
    Serial.println(_offset, HEX);
    Serial.print("ttl: 0x");
    Serial.println(_ttl, HEX);
    Serial.print("type: 0x");
    Serial.println(_type, HEX);
    Serial.print("checksum: 0x");
    Serial.println(_check_sum, HEX);
    Serial.print("from ip: ");
    _from_addr.print();
    Serial.print(" to ip: ");
    _to_addr.print();
    Serial.print("cchecksum: 0x");
    Serial.println(get_calc_check_sum(), HEX);
  }
};

struct icmp_pkg {
  uint8_t _type;
  uint8_t _code;
  uint16_t _check_sum;
  uint16_t _id;
  uint16_t _seq;

  uint16_t get_calc_check_sum() {
    uint16_t old_check_sum = _check_sum;
    _check_sum = 0;
    uint16_t new_check_sum = ip_pkg::check_sum(0, (uint8_t*)this, sizeof(icmp_pkg));
    _check_sum = old_check_sum;
    return new_check_sum;
  }

  bool is_check_sum_valid() {
    return get_calc_check_sum() == _check_sum;
  }

  void print() {
    Serial.print("type: 0x");
    Serial.println(_type, HEX);
    Serial.print("code: 0x");
    Serial.println(_code, HEX);
    Serial.print("check_sum: 0x");
    Serial.println(_check_sum, HEX);
    Serial.print("id: 0x");
    Serial.println(_id, HEX);
    Serial.print("seq: 0x");
    Serial.println(_seq, HEX);
    Serial.print("cchecksum: 0x");
    Serial.println(get_calc_check_sum(), HEX);
  }
};

class in_pkg_handler {
public:
  uint16_t _type;

  in_pkg_handler(uint16_t type):
    _type(type)
  {}

  virtual void read_pkg(uint16_t pos) = 0;
};

class net {
  static const uint8_t ETXSTART = 0x04;
  static const uint8_t ETXEND = 0x06;
  static const uint8_t ERBSTART = 0x08;
  static const uint8_t ERBEND = 0x0A;
  static const uint8_t ERXRDPT = 0x0C;

  static const uint8_t EIR = 0x1C;
  static const uint8_t ECON2 = 0x1E;
  static const uint8_t ECON1 = 0x1F;

  static const uint8_t EIR_TXIF = 0x08;
  static const uint8_t EIR_TXERIF = 0x02;
  static const uint8_t EIR_RXERIF = 0x01;

  static const uint8_t ECON2_PKTDEC = 0x40;

  static const uint8_t ECON1_RXEN = 0x04;
  static const uint8_t ECON1_TXRTS = 0x08;
  static const uint8_t ECON1_TXRST = 0x80;

  static const uint8_t BANK1 = 0x40;
  static const uint8_t ERXFCON = 0x18 | BANK1;
  static const uint8_t EPKTCNT = 0x19 | BANK1;

  static const uint8_t ERXFCON_UCEN = 0x80;
  static const uint8_t ERXFCON_ANDOR = 0x40;
  static const uint8_t ERXFCON_CRCEN = 0x20;
  static const uint8_t ERXFCON_PMEN = 0x10;
  static const uint8_t ERXFCON_MPEN = 0x08;
  static const uint8_t ERXFCON_HTEN = 0x04;
  static const uint8_t ERXFCON_MCEN = 0x02;
  static const uint8_t ERXFCON_BCEN = 0x01;

  static const uint8_t BANK2 = 0x80;
  static const uint8_t MACON1 = 0x00 | BANK2;
  static const uint8_t MACON2 = 0x01 | BANK2;
  static const uint8_t MACON3 = 0x02 | BANK2;
  static const uint8_t MACON4 = 0x03 | BANK2;
  static const uint8_t MABBIPG = 0x04 | BANK2;
  static const uint8_t MAIPG = 0x06 | BANK2;
  static const uint8_t MAMXFL = 0x0A | BANK2;

  static const uint8_t MACON1_INIT = 0x0D;

  static const uint8_t MACON3_FULDPX = 0x01;
  static const uint8_t MACON3_FRMLNEN = 0x02;
  static const uint8_t MACON3_TXCRCEN = 0x10;
  static const uint8_t MACON3_PADCFG = 0x20;

  static const uint16_t FRAMESIZE = 1500;

  static const uint8_t BANK3 = 0xC0;
  static const uint8_t MAADR1 = 0x04 | BANK3;
  static const uint8_t MAADR2 = 0x05 | BANK3;
  static const uint8_t MAADR3 = 0x02 | BANK3;
  static const uint8_t MAADR4 = 0x03 | BANK3;
  static const uint8_t MAADR5 = 0x00 | BANK3;
  static const uint8_t MAADR6 = 0x01 | BANK3;

  static const uint16_t RXSIZE = 0x1A02;

  static const uint8_t PHCON1 = 0x00;
  static const uint8_t PHCON2 = 0x05;
  static const uint8_t PHLCON = 0x14;

  static const uint16_t PHCON1_PDPXMD = 0x0100;
  static const uint16_t PHCON2_HDLDIS = 0x0100;
  static const uint16_t PHLCON_LAONPUT = 0x0100;
  static const uint16_t PHLCON_LBONPUT = 0x0020;
  static const uint16_t PHLCON_MAX_TIME = 0x000A;

  static const uint16_t SEND_BUFF_START = RXSIZE;
    
  uint16_t _next_pkg_ptr;

  in_pkg_handler *_handlers[5];
  uint8_t _handlers_size;
  
public:
  encspi _spi;
  ip_addr _ip;

  net():
    _next_pkg_ptr(0),
    _handlers_size(0)
  {}

  void init(const mac_addr &addr){
    Serial.println("Start init enc28j60 module....");
    _spi.init();

    _spi.write16(ERBSTART, 0);
    _spi.write16(ERBEND, RXSIZE);
    _spi.write16(ERXRDPT, 0);
    _next_pkg_ptr = 0;

    _spi.write(MACON2, 0);
    _spi.write(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN /*| ERXFCON_PMEN*/ | ERXFCON_BCEN);
    _spi.set_bits(MACON1, MACON1_INIT);
    _spi.set_bits(MACON3, MACON3_FULDPX | MACON3_FRMLNEN | MACON3_TXCRCEN | MACON3_PADCFG);  
    _spi.write16(MAMXFL, FRAMESIZE);
    _spi.write(MABBIPG, 0x15);
    _spi.write16(MAIPG, 0x0c12);
    set_mac_addr(addr);

    mac_addr raddr = get_mac_addr();
    Serial.print("mac: ");
    raddr.println();

    _spi.miwrite(PHCON1, PHCON1_PDPXMD);
    _spi.miwrite(PHCON2, PHCON2_HDLDIS);
    _spi.miwrite(PHLCON, PHLCON_LAONPUT | PHLCON_LBONPUT | PHLCON_MAX_TIME);
    _spi.set_bits(ECON1, ECON1_RXEN);

    Serial.print("ERXFCON: ");
    Serial.println(_spi.read(ERXFCON));
    Serial.println();
  }

  void set_mac_addr(const mac_addr &addr) {
    _spi.write(MAADR1, addr._addr[0]);
    _spi.write(MAADR2, addr._addr[1]);
    _spi.write(MAADR3, addr._addr[2]);
    _spi.write(MAADR4, addr._addr[3]);
    _spi.write(MAADR5, addr._addr[4]);
    _spi.write(MAADR6, addr._addr[5]);
  }

  mac_addr get_mac_addr() {
    mac_addr ret;
    ret._addr[0] = _spi.read(MAADR1);
    ret._addr[1] = _spi.read(MAADR2);
    ret._addr[2] = _spi.read(MAADR3);
    ret._addr[3] = _spi.read(MAADR4);
    ret._addr[4] = _spi.read(MAADR5);
    ret._addr[5] = _spi.read(MAADR6);
    return ret;
  }

  void add_read_pkg_handler(in_pkg_handler *handler) {
    _handlers[_handlers_size] = handler;
    _handlers_size++;
  }

  void read_pkg() {
    if (_spi.read(EPKTCNT) <= 0) {
      return;
    }
//    Serial.print(_spi.read(EPKTCNT));
//    Serial.println("+++++++++++++++++++++++ receive pkg +++++++++++++++++++++++");
    enc28j60_header enc28j60_h;
    _spi.read_buff(_next_pkg_ptr, (uint8_t*)&enc28j60_h, sizeof(enc28j60_header));
    if (!enc28j60_h.is_valid()) {
      Serial.println("wrong pkg");
      Serial.println();
      return;
    }
    if (enc28j60_h._frame_len <  sizeof(ethernet_header)) {
      Serial.println("pkg so small");
      Serial.println();
    }
    uint16_t buff_pos = _next_pkg_ptr + sizeof(enc28j60_header);
    ethernet_header ethernet_h;
    _spi.read_buff(buff_pos, (uint8_t*)&ethernet_h, sizeof(ethernet_header));
    buff_pos += sizeof(ethernet_header);
    bool handled = false;
    for(int8_t i = 0; i < _handlers_size; ++i) {
      if (_handlers[i]->_type == ethernet_h._type) {
        _handlers[i]->read_pkg(buff_pos);
        handled = true;
        break;
      }
    }
    _next_pkg_ptr = enc28j60_h._next_pkg_ptr;
    _spi.write16(ERXRDPT, _next_pkg_ptr - 1);
    _spi.set_bits(ECON2, ECON2_PKTDEC);
    if (!handled) {
      Serial.print("Unknowen pkg type: ");
      Serial.print(ethernet_h._type, HEX);
      Serial.println();
//      Serial.println();
    }
  }

  void read_sended() {
    uint8_t f_bute = 0;
    _spi.read_buff_test(SEND_BUFF_START, &f_bute, 1);
    arp_pkg_buff arp_resp;
    _spi.read_buff_test(SEND_BUFF_START + 1, (uint8_t*)&arp_resp, sizeof(arp_pkg_buff));

    Serial.print("First byte: ");
    Serial.println(f_bute);
    Serial.println("Sended pkg: ");
    arp_resp.print();

    Serial.print("Readed: ");
    Serial.println(sizeof(arp_pkg_buff) + 1);
  }

  void write_pkg(uint8_t *buff, uint16_t len) {
    while(_spi.read(ECON1) & ECON1_TXRTS) {};
    _spi.write16(ETXSTART, SEND_BUFF_START);
    uint8_t r_bute = 0;
    _spi.write_buff(SEND_BUFF_START, &r_bute, 1);
    _spi.write_buff(SEND_BUFF_START + 1, buff, len);
    _spi.write16(ETXSTART, SEND_BUFF_START);
    _spi.write16(ETXEND, SEND_BUFF_START + len + 1);
    _spi.set_bits(ECON1, ECON1_TXRTS);
    while(_spi.read(EIR) & (EIR_TXERIF | EIR_RXERIF)) {}
//    Serial.print("Sended: ");
//    Serial.println(len + 1);
//    read_sended();
  }

  void print_buff(uint8_t *buff, uint16_t len) {
    Serial.print("Count - ");
    Serial.print(len);
    Serial.print(": ");
    for(int i = 0; i < len; i++) {
      Serial.print(buff[i], HEX);
    }
    Serial.println();
  }
};

class arp : public in_pkg_handler {
  net *_net;

  struct cache_item {
    ip_addr _ip;
    mac_addr _mac;
  };

  cache_item _cache[5];
  uint8_t _cache_size;  
  
public:
  arp():
    in_pkg_handler(ethernet_header::TYPE_ARP),
    _net(0),
    _cache_size(0)
  {}

  void print_cache() {
    Serial.println("cache:");
    for (int i = 0; i < _cache_size; i++) {
      Serial.print("  ip: " );
      _cache[i]._ip.print();
      Serial.print(" mac: ");
      _cache[i]._mac.println();
    }
  }

  void add_cache(const ip_addr &ip, const mac_addr &mac) {
    for (uint8_t i = 0; i < _cache_size; i++) {
      if (_cache[i]._ip == ip) {
        _cache[i]._mac = mac;
        return;
      }
    }
    _cache[_cache_size]._ip = ip;
    _cache[_cache_size]._mac = mac;
    _cache_size++;
  }

  mac_addr get_mac(const ip_addr &ip) {
    for (uint8_t i = 0; i < _cache_size; i++) {
      if (_cache[i]._ip == ip) {
        return _cache[i]._mac;
      }
    }
    return mac_addr();
  }

  void init(net *n) {
    _net = n;
    _net->add_read_pkg_handler(this);
  }
  
  void send_req(const ip_addr &to_ip, const mac_addr &from_mac, const ip_addr &from_ip) {
    arp_pkg_buff arp_req;
    arp_req.init_req(to_ip, from_mac, from_ip);    
    _net->write_pkg((uint8_t*)&arp_req, sizeof(arp_pkg_buff));
  }

  void send_resp(const mac_addr &to_mac, const ip_addr &to_ip, const mac_addr &from_mac, const ip_addr &from_ip) {
    arp_pkg_buff arp_resp;
    arp_resp.init_resp(to_mac, to_ip, from_mac, from_ip);
    _net->write_pkg((uint8_t*)&arp_resp, sizeof(arp_pkg_buff));
  }

  virtual void read_pkg(uint16_t pos) {
    arp_pkg arp_req;
    _net->_spi.read_buff(pos, (uint8_t*)&arp_req, sizeof(arp_pkg));
    if (!arp_req.is_valid()) {
      Serial.println("arp pkg invalid.");
      Serial.println();
      return;
    }
    if (arp_req._opcode == arp_pkg::OPCODE_REQ) {
      if (arp_req._tpa != _net->_ip) {
//        Serial.print("arp target ip: ");
//        arp_req._tpa.println();
        return;
      }
      send_resp(arp_req._sha, arp_req._spa, _net->get_mac_addr(), _net->_ip);
      Serial.println("answer to arp request.");
      Serial.println();
//      send_request(ip_addr(100, 100, 100, 1));
      return;
    }
    if (arp_req._opcode == arp_pkg::OPCODE_RESP) {
      add_cache(arp_req._spa, arp_req._sha);
      Serial.println("arp response readed.");
      print_cache();
      Serial.println();
      return;
    }
    Serial.println("unknowen arp opcode");
    Serial.println();
  }

  void send_request(const ip_addr &ip) {
    send_req(ip, _net->get_mac_addr(), _net->_ip);
  }
};

class ip : public in_pkg_handler {
  in_pkg_handler *_handlers[5];
  uint8_t _handlers_size;
  
public:
  net *_net;

  ip():
    in_pkg_handler(ethernet_header::TYPE_IP),
    _handlers_size(0),
    _net(0)
  {}

  void init(net *n) {
    _net = n;
    _net->add_read_pkg_handler(this);
  }

  void add_read_pkg_handler(in_pkg_handler *handler) {
    _handlers[_handlers_size] = handler;
    _handlers_size++;
  }

  virtual void read_pkg(uint16_t pos) {
    ip_pkg ip_header;
    _net->_spi.read_buff(pos, (uint8_t*)&ip_header, sizeof(ip_pkg));
    if (!ip_header.is_check_sum_valid()) {
      Serial.print("wrong ip header check sum");
      Serial.println();
      return;
    }
    pos += sizeof(ip_pkg);
    bool handled = false;
    for(int8_t i = 0; i < _handlers_size; ++i) {
      if (_handlers[i]->_type == ip_header._type) {
        _handlers[i]->read_pkg(pos);
        handled = true;
        break;
      }
    }
    if (!handled) {
      Serial.print("Unknowen ip type: ");
      Serial.print(ip_header._type, HEX);
      Serial.println();   
    }
  }
};

class icmp : public in_pkg_handler {
  ip *_ip;

public:
  icmp():
    in_pkg_handler(ip_pkg::TYPE_ICMP),
    _ip(0)
  {}

  void init(ip *n) {
    _ip = n;
    _ip->add_read_pkg_handler(this);
  }

  virtual void read_pkg(uint16_t pos) {
    icmp_pkg icmp_data;
    _ip->_net->_spi.read_buff(pos, (uint8_t*)&icmp_data, sizeof(icmp_pkg));
    icmp_data.print();
    Serial.println();
  }
};

net _net;
arp _arp;
ip _ip;
icmp _icmp;

void setup() {
  Serial.begin(9600);
  _net.init(mac_addr(0x74,0x69,0x69,0x2D,0x30,0x34));
  _net._ip = ip_addr(100, 100, 100, 12);
  _arp.init(&_net);
  _ip.init(&_net);
  _icmp.init(&_ip);
  delay(10);
  _arp.send_request(ip_addr(100, 100, 100, 1));
}

void loop() {
  _net.read_pkg();
  
  // put your main code here, to run repeatedly:
}
