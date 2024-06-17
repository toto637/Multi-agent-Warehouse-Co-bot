import requests

# Define the data you want to send
data = {'data': 'Temi'}

# Define the URL to send the POST request to
url = 'http://192.168.1.4:8000/send_data'

# Send the POST request with the data
response = requests.post(url, data=data, timeout=10)  # Set timeout to 10 seconds


# Print the response from the server
print(response.text)