lang_table = {
["Fly control simulator"] = "�ɿ�ģ����",
["Btn"] = "��",
["Client States"] = "ҡ��״̬",
["Scan here\r\nto get\r\nAndroid\r\nclient"] = "ɨ����\r\n�ذ�׿\r\n�ͻ���",
["&Update"] = "����(&U)",
["&Save"] = "����(&S)",
["&Load"] = "����(&L)",
["Setting"] = "����",
["Preview"] = "Ԥ��",
["Open"] = "��",
["Close"] = "�ر�",
["Refresh"] = "ˢ��",
["Baud:"] = "������:",
["Port:"] = "�˿�:",
["Serial setting"] = "��������",
["TCP Server"] = "TCP�����",
["Listen"] = "����",
["TCP server setting"] = "TCP���������",
["Local Address:"] = "������ַ:",
["TCP client status"] = "TCP�ͷ���״̬",
["IP:"] = "IP:",
["Serial Port Viewer"] = "���ڼ�����",
["success"] = "�ɹ�",
["fail"] = "ʧ��",
["Open online help"] = "�����߰���",
["New connection:"] = "������:",
["Base class:"] = "����:",
["Search Help:"] = "��������:",
["methods"] = "����",
["attributes"] = "����",
["static"] = "��̬����",
["constants"] = "����",
["Class Name Filter:"] = "��������:",
["Name Filter:"] = "���ֹ���:",
["Dynamic Help"] = "��̬����",
["&Help"] = "����(&H)",
["Parity:"] = "У��:",
["Data Bits:"] = "����λ:",
["Stop Bits:"] = "ֹͣλ:",
["Flow:"] = "����:",
["Line status:"] = "״̬:",
["HID Viewer"] = "HID������",
["Serial Viewer"] = "���ڼ�����",
["Serial View"] = "���ڼ�����",
["Send"] = "����",
["Clear"] = "���",
["Send data"] = "��������",
["Recv data"] = "��������",
}

function tr(a)
    if lang_table[a] then
    else
        lang_table[a] = a
        log("δ����:<" .. a .. ">")
    end
    return lang_table[a] or a
end