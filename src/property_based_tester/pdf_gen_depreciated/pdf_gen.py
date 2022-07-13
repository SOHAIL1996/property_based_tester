#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
---------------------------------------------------- 
Report Generator

Alternate version for the report generator.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""
from datetime import date
from reportlab.pdfgen import canvas 
from reportlab.lib.enums import TA_JUSTIFY
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import Paragraph, SimpleDocTemplate, Table,TableStyle
from reportlab.platypus import PageBreak, Spacer, Image
from reportlab.lib import colors
from reportlab.lib.units import inch,mm
from reportlab.pdfbase.ttfonts import TTFont

##############################
# Information
##############################
tick = '\u2713'
cross = '\u2717'
##############################
today = date.today()
today = str(today.strftime("%b-%d-%Y"))
file_name = 'HSR_Report_'+today+'.pdf'
doc_title = 'Report'
title     = 'Toyota HSR-Test Report'
data_1    = [['Navigation Property Tests','','','','Results'],
             ['','','','',''],
             ['Robot pose is same as simulation pose','','','',tick],
             ['Pose of objects are the same after navigation','','','',tick],
             ['Robot nagivation run completed in average time','','','',tick],
             ['','','','',''],
             ['Legality Tests','','','',''],
             ['','','','',''],]
data_2    = [['Data 9','Data 10','Data 11','Data 12',tick],
             ['Data 13','Data 7','Data 8','Data 13',cross]]
data_3    = [['Data 188','Data 2','Data 3','Data 4',tick],
             ['Data 6','Data 7','Data 8','Data 9',cross],] 
##############################
# Alternate
##############################
pdf = canvas.Canvas(file_name)
styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='Justify', alignment=TA_JUSTIFY))
##############################
# PDF_Title
##############################
pdf.setFont('Courier-Bold',32)
pdf.setFillColorRGB(9/255,28/255,87/255)
pdf.setTitle(doc_title)
pdf.drawCentredString(300,690, title)
##############################
# PDF_line
##############################
pdf.line(30,670,550,670)
pdf.line(30,75,550,75)
##############################
# PDF_Text
##############################
# text = pdf.beginText(40,610)
# pdf.setFont('Courier',12)
# for line in text_line:
#     text.textLine(line)
# pdf.drawText(text)

##############################
# PDF_Image
##############################
pdf.drawInlineImage('Images/hbrs.jpg', -70, 770,height=40,preserveAspectRatio=True)
pdf.drawInlineImage('Images/bit.png', -700, 770,height=40,preserveAspectRatio=True)
##############################
# PDF_Table_Properties
##############################
table_1 = Table(data_1, colWidths=[1.3*inch] * 1)
table_2 = Table(data_2, colWidths=[1.3*inch] * 1)
table_3 = Table(data_3, colWidths=[1.3*inch] * 1)
style = TableStyle([
    # ('BACKGROUND', (0,0),(4,0),(135/255,206/255,235/255)),
    # ('BACKGROUND', (0,3),(4,3),(135/255,206/255,235/255)),
    # ('BACKGROUND', (0,6),(4,6),(135/255,206/255,235/255)),
    # ('BACKGROUND', (0,1),(4,2),colors.beige),
    # ('TEXTCOLOR', (0,0),(-1,0),colors.whitesmoke),
    # ('TEXTCOLOR', (0,3),(-1,3),colors.whitesmoke),
    # ('TEXTCOLOR', (0,6),(-1,6),colors.whitesmoke),
    ('ALIGN',(0,0),(-1,-1),'LEFT'),
    ('ALIGN',(4,0),(4,-1),'CENTER'),
    ('FONTNAME',(0,0),(4,0),'Courier-Bold'),
    # ('FONTNAME',(0,3),(4,3),'Courier-Bold'),
    # ('FONTNAME',(0,6),(4,6),'Courier-Bold'),
    ('FONTSIZE',(0,0),(4,0),14),
    # ('FONTSIZE',(0,3),(-1,3),16),
    # ('FONTSIZE',(0,6),(-1,6),16),
    # ('BOTTOMPADDING',(0,0),(-1,0),12),
    # ('BOTTOMPADDING',(0,3),(-1,3),12),
    # ('BOTTOMPADDING',(0,6),(-1,6),12),
])

border= TableStyle([
    ('BOX',(0,0),(-1,-1),2,(255,255,255)),
    # ('LINEBEFORE',(2,1),(2,-1),2,colors.white), # Horizontal Line
    # ('LINEABOVE',(0,2),(-1,2),2,colors.white), # Veritcal Line
    ('GRID',(0,1),(-1,2),2,colors.white),
    ('GRID',(0,4),(-1,5),2,colors.white),
    ('GRID',(0,7),(-1,8),2,colors.white),
])
##############################
# PDF table and header
##############################
def pdf_header_table(table,header_posx,header_posy,table_posx,table_posy,
                     text,font_size,border,style,color_r,color_g,color_b):
    """Creates a table for the pdf.
    Args:
        table ([list]): Data list.
        header_posx ([int]): 
        header_posy ([int]): 
        table_posx ([int]): 
        table_posy ([int]): 
        text ([str]): 
        font_size ([int]): 
        border ([list]): 
        style ([list]): 
        color_r ([int]): 
        color_g ([int]): 
        color_b ([int]): 
    """    
    pdf.setFillColorRGB(color_r/255,color_g/255,color_b/255)
    pdf.setFont('Courier-Bold',font_size)
    pdf.drawCentredString(header_posx, header_posy, text)
    # table.setStyle(border)
    table.setStyle(style)
    table.wrapOn(pdf, 700, 700)
    table.drawOn(pdf, table_posx, table_posy)
    
pdf_header_table(table_1,2.4*inch,620,1*inch, 400,'Evaluation of Navigation',16,border,style,0,120,255)
pdf.showPage()
pdf_header_table(table_2,2.4*inch,620,1*inch, 400,'Evaluation of Pick Action',16,border,style,0,120,255)
pdf.showPage()
pdf_header_table(table_3,2.4*inch,620,1*inch, 400,'Evaluation of Place Action',16,border,style,0,120,255)
pdf.showPage()
pdf_header_table(table_3,1.6*inch,220,1*inch, 170,'Evaluation of Scenario',16,border,style,0,120,255)
############################## 
# Text and Image
############################## 
paragraph_1 = Paragraph(title, styles['Heading1'])
paragraph_2 = Paragraph("Some normal body text", styles['BodyText'])
# Paragraph("A <b>bold</b> word.<br /> An <i>italic</i> word.", custom_body_style)
im1 = Image('Images/hbrs.jpg',width=3*inch,height=1*inch,hAlign='RIGHT')
im2 = Image('Images/bit.png',width=3*inch,height=1*inch,hAlign='LEFT')
ptext = '''
<seq>. </seq>Some Text<br/>
<seq>. </seq>Some more test Text
'''
pt = Paragraph(ptext, styles["Bullet"])

pdf.save()
##############################  