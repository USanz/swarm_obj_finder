


def ser_string(s, s_lenght, fill_char=' '):
    ns_bytes = bytes(s, "utf-8")
    #Fixed ns lenght of s_lenght (fill with space characters):
    return bytes(fill_char, "utf-8") * (s_lenght - len(ns_bytes)) + ns_bytes

def deser_string(bytes, fill_chars=' '):
    return str(bytes.decode('utf-8')).lstrip(fill_chars)
    