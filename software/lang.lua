lang_table = {
["Fly control simulator"] = "飞控模拟器",
["Btn"] = "键",
["Client States"] = "摇杆状态",
["Scan here\r\nto get\r\nAndroid\r\nclient"] = "扫描下\r\n载安卓\r\n客户端",
["&Update"] = "更新(&U)",
["&Save"] = "保存(&S)",
["&Load"] = "加载(&L)",
["Setting"] = "设置",
["Preview"] = "预览",
["Open"] = "打开",
["Close"] = "关闭",
["Refresh"] = "刷新",
["Baud:"] = "波特率:",
["Port:"] = "端口:",
["Serial setting"] = "串口设置",
["TCP Server"] = "TCP服务端",
["Listen"] = "监听",
["TCP server setting"] = "TCP服务端设置",
["Local Address:"] = "本机地址:",
["TCP client status"] = "TCP客服端状态",
["IP:"] = "IP:",
["Serial Port Viewer"] = "串口监视器",
["success"] = "成功",
["fail"] = "失败",
["Open online help"] = "打开在线帮助",
["New connection:"] = "新连接:",
["Base class:"] = "基类:",
["Search Help:"] = "搜索帮助:",
["methods"] = "方法",
["attributes"] = "属性",
["static"] = "静态函数",
["constants"] = "常量",
["Class Name Filter:"] = "类名过滤:",
["Name Filter:"] = "名字过滤:",
["Dynamic Help"] = "动态帮助",
["&Help"] = "帮助(&H)",
["Parity:"] = "校验:",
["Data Bits:"] = "数据位:",
["Stop Bits:"] = "停止位:",
["Flow:"] = "流控:",
["Line status:"] = "状态:",
["HID Viewer"] = "HID监视器",
["Serial Viewer"] = "串口监视器",
["Serial View"] = "串口监视器",
["Send"] = "发送",
["Clear"] = "清除",
["Send data"] = "发送数据",
["Recv data"] = "接收数据",
}

function tr(a)
    if lang_table[a] then
    else
        lang_table[a] = a
        log("未翻译:<" .. a .. ">")
    end
    return lang_table[a] or a
end