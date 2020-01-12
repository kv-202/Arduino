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

  bool is_empty() const {
    return !_addr[0] && !_addr[1] && !_addr[2] && !_addr[3] && !_addr[4] && !_addr[5]; 
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

  void init_req(const ip_addr &to_ip_addr, const mac_addr &from_mac_addr, const ip_addr &from_ip_addr) {
    init();
    _opcode = arp_pkg::OPCODE_REQ;
    _sha = from_mac_addr;
    _spa = from_ip_addr;
    _tpa = to_ip_addr;
  }

  void init_resp(const mac_addr &to_mac_addr, const ip_addr &to_ip_addr, const mac_addr &from_mac_addr, const ip_addr &from_ip_addr) {
    init();
    _opcode = arp_pkg::OPCODE_RESP;
    _sha = from_mac_addr;
    _spa = from_ip_addr;
    _tha = to_mac_addr;
    _tpa = to_ip_addr;
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

  void init() {
    set_ver(4);
    set_ihl(5);
    set_offset(0);
    _ttl = 0x80;
  }

  uint8_t get_ver() const { return (_ver & VER_MASK) >> 4; }

  uint8_t get_ihl() const { return (_ver & IHL_MASK); }

  uint16_t get_len() const { return htons(_len); }

  uint16_t get_id() const { return htons(_id); }

  void set_ver(uint8_t value) { _ver &= IHL_MASK; _ver |= (value << 4); }

  void set_ihl(uint8_t value) { _ver &= VER_MASK; _ver |= (value & IHL_MASK); }

  void set_len(uint16_t value) { _len = htons(value); }

  void set_id(uint16_t value) { _id = htons(value); }

  void set_offset(uint16_t value) { _offset = htons(value); }

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

  void print() const {
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
    _to_addr.println();
  }
};

struct icmp_pkg {
  static const uint8_t TYPE_ECHO_REQ = 0x08;
  static const uint8_t TYPE_ECHO_RESP = 0x00;

  static const uint8_t CODE_ECHO = 0x00;
  
  uint8_t _type;
  uint8_t _code;
  uint16_t _check_sum;
  uint16_t _id;
  uint16_t _seq;

  static uint16_t get_calc_check_sum(uint8_t *buff, uint16_t len) {
    icmp_pkg *icmp_data = (icmp_pkg*)buff;
    uint16_t old_check_sum = icmp_data->_check_sum;
    icmp_data->_check_sum = 0;
    uint16_t new_check_sum = ip_pkg::check_sum(0, buff, len);
    icmp_data->_check_sum = old_check_sum;
    return new_check_sum;
  }

  static bool is_check_sum_valid(uint8_t *buff, uint16_t len) {
    icmp_pkg *icmp_data = (icmp_pkg*)buff;
    return icmp_data->_check_sum == get_calc_check_sum(buff, len);
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
  }
};

class pkg_handler {
public:
  uint16_t _type;

  pkg_handler(uint16_t type):
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

  static const uint8_t HANDLERS_CAPACITY = 5;
    
  uint16_t _next_pkg_ptr;
  uint16_t _send_pkg_pos;

  pkg_handler *_handlers[HANDLERS_CAPACITY];
  uint8_t _handlers_size;

public:
  encspi _spi;
  ip_addr _ip;

  net():
    _next_pkg_ptr(0),
    _send_pkg_pos(0),
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

    Serial.print("ip: ");
    _ip.println();
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

  bool add_read_pkg_handler(pkg_handler *handler) {
    if (_handlers_size >= HANDLERS_CAPACITY) {
      return false;
    }
    _handlers[_handlers_size] = handler;
    _handlers_size++;
    return true;
  }

  void read_pkg() {
    if (_spi.read(EPKTCNT) <= 0) {
      return;
    }
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
    }
  }

  void read_sended() {
    uint8_t f_bute = 0;
    _spi.read_buff_test(SEND_BUFF_START, &f_bute, 1);
//    arp_pkg_buff arp_resp;
//    _spi.read_buff_test(SEND_BUFF_START + 1, (uint8_t*)&arp_resp, sizeof(arp_pkg_buff));

    Serial.print("First byte: ");
    Serial.println(f_bute);
    Serial.println("Sended pkg: ");
//    arp_resp.print();

    Serial.print("Readed: ");
//    Serial.println(sizeof(arp_pkg_buff) + 1);
  }

  void write_pkg(uint8_t *buff, uint16_t len) {
    while(_spi.read(ECON1) & ECON1_TXRTS) {};
    uint8_t r_bute = 0;
    _spi.write_buff(SEND_BUFF_START, &r_bute, 1);
    _spi.write_buff(SEND_BUFF_START + 1, buff, len);
    _spi.write16(ETXSTART, SEND_BUFF_START);
    _spi.write16(ETXEND, SEND_BUFF_START + len + 1);
    _spi.set_bits(ECON1, ECON1_TXRTS);
    while(_spi.read(EIR) & (EIR_TXERIF | EIR_RXERIF)) {}
  }

  void begin_send_pkg(const mac_addr &from, const mac_addr &to, uint16_t type) {
    while(_spi.read(ECON1) & ECON1_TXRTS) {};
    uint8_t r_bute = 0;
    _spi.write_buff(SEND_BUFF_START, &r_bute, 1);
    ethernet_header pkg;
    pkg._to_addr = to;
    pkg._from_addr = from;
    pkg._type = type;
    _spi.write_buff(SEND_BUFF_START + 1, (uint8_t*)&pkg, sizeof(ethernet_header));
    _send_pkg_pos = SEND_BUFF_START + 1 + sizeof(ethernet_header);
  }

  void send_pkg(uint8_t *buff, uint16_t len) {
    _spi.write_buff(_send_pkg_pos, buff, len);
    _send_pkg_pos += len;
  }

  void end_send_pkg(uint8_t *buff, uint16_t len) {
    _spi.write_buff(_send_pkg_pos, buff, len);
    _spi.write16(ETXSTART, SEND_BUFF_START);
    _spi.write16(ETXEND, _send_pkg_pos + len);
    _spi.set_bits(ECON1, ECON1_TXRTS);
    while(_spi.read(EIR) & (EIR_TXERIF | EIR_RXERIF)) {}
    _send_pkg_pos = 0;
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

struct arp_cache_item {
  ip_addr _ip;
  mac_addr _mac;
};

class arp_handler {
public:
  virtual void ip_resolved(const ip_addr &addr) = 0;
};

class arp : public pkg_handler {
  static const uint8_t HANDLER_CAPACITY = 1;
  static const uint8_t CACHE_CAPACITY = 20;
  
  net *_net;
  arp_handler *_handler[HANDLER_CAPACITY];
  uint8_t _handler_capacity;
  arp_cache_item _cache[CACHE_CAPACITY];
  uint8_t _cache_size;
  
public:
  arp():
    pkg_handler(ethernet_header::TYPE_ARP),
    _net(0),
    _handler_capacity(0),
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

  bool add_handler(arp_handler *handler) {
    if (_handler_capacity >= HANDLER_CAPACITY) {
      return false;
    }
    _handler[_handler_capacity] = handler;
    ++_handler_capacity;
    return true;
  }

  bool add_cache(const ip_addr &ip, const mac_addr &mac) {
    if (_cache_size >= CACHE_CAPACITY) {
      return false; 
    }
    for (uint8_t i = 0; i < _cache_size; i++) {
      if (_cache[i]._ip == ip) {
        _cache[i]._mac = mac;
        return true;
      }
    }
    _cache[_cache_size]._ip = ip;
    _cache[_cache_size]._mac = mac;
    _cache_size++;
    return true;
  }

  bool del_cache(const ip_addr &ip) {
    if (_cache_size <= 0) {
      return false;
    }
    uint8_t move = 0;
    for (uint8_t i = 0; i < _cache_size; i++) {
      if (_cache[i]._ip == ip) {
        ++move;
      }
      else
      {
        if (move > 0) {
          _cache[i - move] = _cache[i];
        }
      }
    }
    _cache_size -= move;
    return move;
  }

  mac_addr get_mac(const ip_addr &ip) {
    for (uint8_t i = 0; i < _cache_size; i++) {
      if (_cache[i]._ip == ip) {
        return _cache[i]._mac;
      }
    }
    _net->begin_send_pkg(_net->get_mac_addr(), mac_addr(0xff, 0xff, 0xff, 0xff, 0xff, 0xff), ethernet_header::TYPE_ARP);
    arp_pkg arp_req;
    arp_req.init_req(ip, _net->get_mac_addr(), _net->_ip);
    _net->end_send_pkg((uint8_t*)&arp_req, sizeof(arp_pkg));
    return mac_addr();
  }

  void init(net *n) {
    _net = n;
    _net->add_read_pkg_handler(this);
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
        return;
      }
      _net->begin_send_pkg(_net->get_mac_addr(), arp_req._sha, ethernet_header::TYPE_ARP);
      arp_pkg arp_resp;
      arp_resp.init_resp(arp_req._sha, arp_req._spa, _net->get_mac_addr(), _net->_ip);
      _net->end_send_pkg((uint8_t*)&arp_resp, sizeof(arp_pkg));
      Serial.println();
      return;
    }
    if (arp_req._opcode == arp_pkg::OPCODE_RESP) {
      add_cache(arp_req._spa, arp_req._sha);
      for (int i = 0; i < _handler_capacity; ++i) {
        _handler[i]->ip_resolved(arp_req._spa);
      }
      return;
    }
    Serial.println("unknowen arp opcode");
    Serial.println();
  }
};

#define POS_TYPE uint32_t 
struct send_ip_pkg_item {
  static const uint8_t P_NEXT_BEGIN = 0x01;
  static const uint8_t P_USED = 0x02;

  uint16_t _buff_len;
  ip_addr _ip;
  uint8_t _type;
  uint16_t _id;
  uint8_t _prop;

  void init(uint16_t buff_len, const ip_addr& addr, uint8_t type, uint16_t id, uint8_t prop) {
      _buff_len = buff_len;
      _ip = addr;
      _type = type;
      _id = id;
      _prop = prop;
  }

  uint16_t get_size() const {
    return sizeof(send_ip_pkg_item) + _buff_len;
  }

  POS_TYPE get_next_pos(POS_TYPE pos, uint16_t buff_size) const {
    if (_prop & P_NEXT_BEGIN) {
      return 0;
    }
    POS_TYPE next_pos = pos + get_size();
    if (next_pos > buff_size) {
      return 0;
    }
    return next_pos;
  }

  void write_buff(uint8_t *buff, uint16_t len) {
    uint8_t *this_buff = (uint8_t*)(this + 1); 
    for (uint16_t i = 0; i < len; ++i) {
      this_buff[i] = buff[i];
    }
  }
};

class ip_handler {
public:
  uint16_t _type;

  ip_handler(uint16_t type):
    _type(type)
  {}

  virtual void read_pkg(uint16_t pos, const ip_pkg &ip_header) = 0;
};

class ip : public pkg_handler, arp_handler {
  static const uint16_t BUFF_SIZE = 300;
  static const uint8_t HANDLERS_CAPACITY = 5;
  
  ip_handler *_handlers[HANDLERS_CAPACITY];
  uint8_t _handlers_size;

  uint8_t _buff[BUFF_SIZE];
  bool _empty_buff;
  POS_TYPE _first_item_pos;
  POS_TYPE _last_item_pos;

  send_ip_pkg_item* get_item(uint16_t pos) {
    return (send_ip_pkg_item*)(_buff + pos);
  }
  
  bool save_ip_pkg_data(const ip_addr& addr, uint8_t type, uint16_t ip_id, uint8_t* buff, uint16_t len) {
    uint16_t new_item_len = sizeof(send_ip_pkg_item) + len;
    if (new_item_len > BUFF_SIZE) {
      return false;
    }
    send_ip_pkg_item* last_item = get_item(_last_item_pos);
    if (!_empty_buff) {
      _last_item_pos = last_item->get_next_pos(_last_item_pos, BUFF_SIZE);
      uint16_t last_item_end_pos = _last_item_pos + new_item_len;
      if ((last_item_end_pos) > BUFF_SIZE) {
        last_item->_prop |= send_ip_pkg_item::P_NEXT_BEGIN;
        _last_item_pos = 0;
        _first_item_pos = get_item(0)->get_next_pos(0, BUFF_SIZE);
        last_item_end_pos = new_item_len;
      }
      if (_first_item_pos > _last_item_pos) {
        while (_first_item_pos && _first_item_pos < last_item_end_pos) {
          _first_item_pos = get_item(_first_item_pos)->get_next_pos(_first_item_pos, BUFF_SIZE);
        }
      }
      last_item = get_item(_last_item_pos);
    }
    last_item->init(len, addr, type, ip_id, 0);
    last_item->write_buff(buff, len);
    _empty_buff = false;
    return true;
  }

  void send_saved_pkg(const ip_addr &addr) {
    if (_empty_buff) {
      return;
    }
    uint16_t next_pos = _first_item_pos;
    for (;;) {
      send_ip_pkg_item *item = get_item(next_pos);
      if (!(item->_prop & send_ip_pkg_item::P_USED) || (item->_ip != addr)) {
        mac_addr to_mac_addr = _arp->get_mac(addr);
        if (to_mac_addr.is_empty()) {
          Serial.println("can't resolv ip addres");
        }
        else
        {
          uint8_t *buff = (uint8_t*)(item + 1);
          send_pkg(item->_ip, to_mac_addr, item->_type, item->_id, buff, item->_buff_len);
        }      
        item->_prop |= send_ip_pkg_item::P_USED;
      }
      if (_last_item_pos == next_pos) {
        break;
      }
      next_pos = item->get_next_pos(next_pos, BUFF_SIZE);
    }
  }

  void send_pkg(const ip_addr &addr, const mac_addr &to_mac_addr, uint8_t type, uint16_t ip_id, uint8_t *buff, uint16_t len) {
    _net->begin_send_pkg(_net->get_mac_addr(), to_mac_addr, ethernet_header::TYPE_IP);
    ip_pkg ip_header;
    ip_header.init();
    ip_header.set_id(ip_id);
    ip_header.set_len(sizeof(ip_pkg) + len);
    ip_header._type = type;
    ip_header._from_addr = _net->_ip;
    ip_header._to_addr = addr;
    ip_header._check_sum = ip_header.get_calc_check_sum();
    _net->send_pkg((uint8_t*)&ip_header, sizeof(ip_pkg));
    _net->end_send_pkg(buff, len);
  }
  
public:
  net *_net;
  arp *_arp;

  ip():
    pkg_handler(ethernet_header::TYPE_IP),
    _handlers_size(0),
    _empty_buff(true),
    _first_item_pos(0),
    _last_item_pos(0),
    _net(0),
    _arp(0)
  {}

  void init(net *n, arp *a) {
    _net = n;
    _net->add_read_pkg_handler(this);
    _arp = a;
    _arp->add_handler(this);
  }

  bool add_read_pkg_handler(ip_handler *handler) {
    if (_handlers_size >= HANDLERS_CAPACITY) {
      return false;
    }
    _handlers[_handlers_size] = handler;
    _handlers_size++;
    return true;
  }

  virtual void ip_resolved(const ip_addr &addr) {
    send_saved_pkg(addr);
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
        _handlers[i]->read_pkg(pos, ip_header);
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

  bool send_ip_pkg(const ip_addr &to_ip_addr, uint8_t type, uint16_t ip_id, uint8_t *buff, uint16_t len) {
    mac_addr to_mac_addr = _arp->get_mac(to_ip_addr);
    if (to_mac_addr.is_empty()) {
      if (!save_ip_pkg_data(to_ip_addr, type, ip_id, buff, len)) {
        Serial.print("can't save ip pkg data.");
        Serial.println();
        return false;       
      }
      return true;
    }
    send_pkg(to_ip_addr, to_mac_addr, type, ip_id, buff, len);
    return true;
  }
};

class icmp : public ip_handler {
  const static uint16_t EXHO_REQ_BUFF_SIZE = 60 - sizeof(ip_pkg);
  
  ip *_ip;

public:
  icmp():
    ip_handler(ip_pkg::TYPE_ICMP),
    _ip(0)
  {}

  void init(ip *n) {
    _ip = n;
    _ip->add_read_pkg_handler(this);
  }

  virtual void read_pkg(uint16_t pos, const ip_pkg &ip_header) {
    if (ip_header.get_len() != (EXHO_REQ_BUFF_SIZE + sizeof(ip_pkg))) {
      Serial.println("wrong icmp len.");
      Serial.println();
      return;
    }
    uint8_t buff[EXHO_REQ_BUFF_SIZE];
    _ip->_net->_spi.read_buff(pos, buff, EXHO_REQ_BUFF_SIZE);
    icmp_pkg *icmp_data = (icmp_pkg*)buff;
    if (!icmp_pkg::is_check_sum_valid(buff, EXHO_REQ_BUFF_SIZE)) {
      Serial.println("wrong icmp echo request check sum.");
      Serial.println();
      return;
    }
    if ((icmp_data->_type != icmp_pkg::TYPE_ECHO_REQ) || (icmp_data->_code != icmp_pkg::CODE_ECHO)) {
      return;
    }   
    icmp_data->_type = icmp_pkg::TYPE_ECHO_RESP;
    icmp_data->_check_sum = icmp_pkg::get_calc_check_sum(buff, EXHO_REQ_BUFF_SIZE);
    _ip->send_ip_pkg(ip_header._from_addr, ip_pkg::TYPE_ICMP, ip_header.get_id() + 1, buff, EXHO_REQ_BUFF_SIZE);
  }
};

net _net;
arp _arp;
ip _ip;
icmp _icmp;

void setup() {
  Serial.begin(9600);
  _net._ip = ip_addr(100, 100, 100, 12);
  _net.init(mac_addr(0x74,0x69,0x69,0x2D,0x30,0x34));
  _arp.init(&_net);
  _ip.init(&_net, &_arp);
  _icmp.init(&_ip);
  delay(10);
}

void loop() {
  _net.read_pkg();
}
