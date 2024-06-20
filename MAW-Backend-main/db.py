import sqlite3

def create_table():
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    c.execute("""CREATE TABLE IF NOT EXISTS requests
                 (id INTEGER PRIMARY KEY, location TEXT, destination TEXT, rid INTEGER)""")
    conn.commit()
    conn.close()

def insert_request(id, location):
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    c.execute("INSERT INTO requests(id, location) VALUES (?, ?)", (id, location))
    conn.commit()
    conn.close()

def get_request(id):
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    c.execute("SELECT * FROM requests WHERE id=?", (id,))
    data = c.fetchone()
    conn.close()
    return data

def update_dest(id, destination):
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    c.execute("UPDATE requests SET destination=? WHERE id=?", (destination, id))
    conn.commit()
    conn.close()

def update_rid(id, rid):
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    c.execute("UPDATE requests SET rid=? WHERE id=?", (rid, id))
    conn.commit()
    conn.close()

def clear_request(id):
    conn = sqlite3.connect('data.db')
    c = conn.cursor()
    c.execute("DELETE FROM requests WHERE id=?", (id,))
    conn.commit()
    conn.close()

#create_table()