require("hidparamview")
require("candata")
require("canbaud")

class "CanChannelView"(QFrame)
function CanChannelView:__init(id, dev)
    QFrame.__init(self)
    self.id = id    
    self.baudParam = CanBaudView()
    self.idBox = QLineEdit { text = "0", inputMask = "HHHHHHHH", maxW = 60}
    self.idExt = QCheckBox(tr("Extend"))
    self.chkRemote = QCheckBox(tr("Remote"))
    self.btnSend = QPushButton(tr("Send"))
    self.btnClear = QPushButton(tr("Clear"))
    self.dataBox = QLineEdit { text = "0", inputMask = "HH,HH,HH,HH,HH,HH,HH,HH", minW = 160}
    self.recvTable = QTableWidget()
    self.recvTable.columnCount = 6
    self.recvTable.hHeader = {tr("ID"), tr("Ext"), tr("Remote"), tr("Data"), tr("Length"), tr("timestamp")}
    self.recvTable:setColumnWidth(0,80)
    self.recvTable:setColumnWidth(1,60)
    self.recvTable:setColumnWidth(2,60)
    self.recvTable:setColumnWidth(3,250)
    self.recvTable:setColumnWidth(4,60)
    self.recvTable:setColumnWidth(4,100)
    self.layout = QVBoxLayout{
        QGroupBox(tr("Parameter Setting")){
          layout = QVBoxLayout{
          self.baudParam}
        },
        QHBoxLayout{ QLabel("ID: 0x"),self.idBox, self.idExt,self.chkRemote,
        QLabel("Data: "),self.dataBox, self.btnSend},
        QHBoxLayout{QLabel("Recieved:"), self.btnClear, QLabel(""), strech = "0,0,1"},
        self.recvTable
    }
    
    self.baudParam.setData = function(x, baud, pres, bs1, bs2, sjw, mode,  res)
        local d = QUtil.fromUint32(baud)
        d = QUtil.fromUint16(d, pres)
        d = QUtil.fromUint8(d, bs1)
        d = QUtil.fromUint8(d, bs2)
        d = QUtil.fromUint8(d, sjw)
        d = QUtil.fromUint8(d, mode)
        d = QUtil.fromUint8(d, res and 1 or 0)
        dev:setParam(self.id, d)
    end
    self.btnSend.clicked = function()
        local d = {}
        string.gsub(self.dataBox.text, "%x%x", function(c)
            d[#d+1] = tonumber("0x" .. c)
        end)
        local cd = CanData( tonumber("0x"..self.idBox.text), d, self.idExt.checked, self.chkRemote.checked)
        dev:sendFrame(self.id, cd:toData())
    end
    self.btnClear.clicked = function()
        while self.recvTable.rowCount > 0 do
            self.recvTable:removeRow(0)
        end
    end
end

function CanChannelView:frameRecv(data, offset)
    offset = offset or 1
    local cd = CanData()
    cd:fromData(data, offset)
    local pos = self.recvTable.rowCount
    self.recvTable:insertRow(pos)
    self.recvTable:setRowHeight(pos,18)
    self.recvTable:setItem(pos, 0, QTableWidgetItem(string.format("%x",cd.id)))
    self.recvTable:setItem(pos, 1, QTableWidgetItem(tostring(cd.extend)))
    self.recvTable:setItem(pos, 2, QTableWidgetItem(tostring(cd.remote)))
    self.recvTable:setItem(pos, 3, QTableWidgetItem(QUtil.showBytes(cd.data)))
    self.recvTable:setItem(pos, 4, QTableWidgetItem(tostring(cd.len)))
    self.recvTable:setItem(pos, 5, QTableWidgetItem(string.format("%d:%03d:%d", cd.ts_s,cd.ts_ms,cd.ts_bs)))
    --log(cd)
end

class "CanView"(QFrame)
function CanView:__init(vid, pid)
    QFrame.__init(self)
    self.vid = vid or 0
    self.pid = pid or 0
    self.hidParam = HIDParamView(self.vid, self.pid)
    self.tab = QTabWidget()
    self.chn1 = CanChannelView(1,self)
    self.chn2 = CanChannelView(2,self)
    self.tab:addTab(self.chn1, tr("Chn1"))
    self.tab:addTab(self.chn2, tr("Chn2"))
    self.layout = QVBoxLayout{
        self.hidParam,
        self.tab
    }
    
    self.hidParam.hid.readyRead = function()
        local r = self.hidParam.hid:readAll()
        --log("Get " .. #r .. " bytes")
        --log(QUtil.showBytes(r))
        if r[2] == 2 then -- frame type
            if r[1] == 1 then
                self.chn1:frameRecv(r, 5)
            elseif r[1] == 2 then
                self.chn2:frameRecv(r, 5)
            end
        else
            log("Get:"..QUtil.showBytes(r))
        end
    end
end

function CanView:sendFrame(id, data)
   --log(id, QUtil.showBytes(data))
   --if id == 1 then
   --    self.chn1:frameRecv(data, 1)
   --elseif id == 2 then
   --    self.chn2:frameRecv(data, 1)
   --end
   local d = {id, 2, 0, 0}
   for i=1,#data do
        d[#d+1] = data[i]
    end
   local r = self.hidParam.hid:writeData(0, d)
   --log(id, QUtil.showBytes(d), "r = ", r)
end

function CanView:setParam(id, data)
    local d = {id, 1, 0, 0}
    for i=1,#data do
        d[#d+1] = data[i]
    end
    local r = self.hidParam.hid:writeData(0, d)
    log(id, QUtil.showBytes(d), "r = ", r)
end

--mdiArea:addSubWindow(CanView()):show()