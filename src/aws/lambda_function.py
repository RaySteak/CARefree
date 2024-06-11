import boto3
from decimal import Decimal
from datetime import datetime
import struct
import json

def float_to_bytes(floats):
    return [struct.pack('f', f) for f in floats]

def bytes_to_float(bytes_list):
    return [struct.unpack('f', b)[0] for b in bytes_list]

def lambda_handler(event, context):
    dynamodb = boto3.resource('dynamodb')
    table = dynamodb.Table('DeviceWeights')
    iot_client = boto3.client('iot-data', region_name='eu-north-1')  
    
    print("hello world")
    
    # Decode the incoming MQTT message (which is in bytes)
    hex_string = event['hex_string']
    incoming_weights_bytes = bytes(hex_string, 'latin-1')
    
    # If the incoming message is in string format, convert it to bytes
    if isinstance(incoming_weights_bytes, str):
        incoming_weights_bytes = bytes(incoming_weights_bytes, 'utf-8')  # or 'utf-8' depending on encoding
    
    current_time = datetime.utcnow().isoformat()

    # Convert bytes to floats for processing
    incoming_weights = bytes_to_float([incoming_weights_bytes[i:i+4] for i in range(0, len(incoming_weights_bytes), 4)])
    
    print(incoming_weights)
    
    # Convert weights to byte code for storage
    byte_weights = float_to_bytes(incoming_weights)

    # Convert byte weights to string for DynamoDB compatibility
    byte_weights = [Decimal(int.from_bytes(v, 'big')) for v in byte_weights]

    # Get the current weights from DynamoDB
    scan_response = table.scan()
    items = scan_response['Items']
    
    if items:
        # Assuming only one item for simplicity; extend logic for multiple items as needed
        existing_item = items[0]
        existing_weights = bytes_to_float([int(v).to_bytes((int(v).bit_length() + 7) // 8, 'big') for v in existing_item['Weights']])
        
        # Calculate the average weights
        average_weights = [(incoming_weights[i] + existing_weights[i]) / 2 for i in range(len(incoming_weights))]
    else:
        average_weights = incoming_weights

    # Convert average weights back to byte code for storage
    average_byte_weights = float_to_bytes(average_weights)
    average_byte_weights = [Decimal(int.from_bytes(v, 'big')) for v in average_byte_weights]
    
    # Update the device-specific weights and increment step counter
    response = table.update_item(
        Key={'DeviceId': 'global'},  # Use a specific key to store the global weights
        UpdateExpression='SET #ts = :val, #timestamp = :time ADD #sc :inc',
        ExpressionAttributeNames={
            '#ts': 'Weights',
            '#timestamp': 'Timestamp',
            '#sc': 'StepCounter'  # Alias for 'StepCounter'
        },
        ExpressionAttributeValues={
            ':val': byte_weights,
            ':time': current_time,
            ':inc': Decimal('1')  # Assuming StepCounter is a numeric type, use Decimal to be compatible with DynamoDB number types
        },
        ReturnValues="UPDATED_NEW"
    )

    # Publish the average weights to MQTT
    iot_client.publish(
        topic='test/globalweights',
        payload=float_to_bytes(average_weights)
    )

    print("Publish")
    
    return {
        'statusCode': 200,
        'body': 'Processed successfully'
    }
