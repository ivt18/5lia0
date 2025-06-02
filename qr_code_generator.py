import qrcode

data = "follow"

qr = qrcode.QRCode(
    box_size=10,
    border=5,
    error_correction=qrcode.constants.ERROR_CORRECT_H
)
qr.add_data(data)
qr.make(fit=True)

img = qr.make_image()
img.save("QRCodes/" + data + ".png")
