#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
---------------------------------------------------- 
Report Generator

Generates a pdf
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

class PdfGenerator():
    """
    This class creates a pdf evaluation sheet of the results.
    """    
    def __init__(self,test_type):
        
        self.tick = '\u2713'
        self.cross = '\u2717'
        
        self.robot_pos_same_as_sim = self.cross
        self.object_pose_same_after_nav = self.cross
        self.robot_nav_time = self.cross
        self.obstacles_placed = '0'
        
        self.query_1 = '-'
        self.query_2 = '-'
        self.query_3 = '-'
        self.query_4 = '-'
        self.query_5 = '-'
        self.query_6 = '-'
        self.query_7 = '-'
        self.query_8 = '-'
        self.query_9 = '-'
        self.query_10 = '-'
        self.query_11 = '-'
        self.query_12 = '-'
        self.query_13 = '-'
        self.query_14 = '-'
        self.query_15 = '-'
        
        self.result_1 = '-'
        self.result_2 = '-'
        self.result_3 = '-'
        self.result_4 = '-'
        self.result_5 = '-'
        self.result_6 = '-'
        self.result_7 = '-'
        self.result_8 = '-'
        self.result_9 = '-'
        self.result_10 = '-'
        self.result_11 = '-'
        self.result_12 = '-'
        self.result_13 = '-'
        self.result_14 = '-'
        self.result_15 = '-'
        
        self.test_type  = test_type
        self.today      = date.today()
        self.result_fol = 'tests/results/'
        self.today      = str(self.today.strftime("%b-%d-%Y"))
        self.file_name  = 'HSR '+test_type+' Report '+self.today+'.pdf'
        self.doc_title  = 'Report'
        self.title      = 'Toyota HSR-Test Report'        
        
    def pdf_header_table(self,table,header_posx,header_posy,table_posx,table_posy,
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
        self.pdf.setFillColorRGB(color_r/255,color_g/255,color_b/255)
        self.pdf.setFont('Courier-Bold',font_size)
        self.pdf.drawCentredString(header_posx, header_posy, text)
        # self.table.setStyle(border)
        table.setStyle(style)
        table.wrapOn(self.pdf, 700, 700)
        table.drawOn(self.pdf, table_posx, table_posy)
            
    def pdf_creation(self):

        self.pdf = canvas.Canvas(self.result_fol+self.file_name)
        self.styles = getSampleStyleSheet()
        self.styles.add(ParagraphStyle(name='Justify', alignment=TA_JUSTIFY))

        # PDF_Title
        self.pdf.setFont('Courier-Bold',32)
        self.pdf.setFillColorRGB(9/255,28/255,87/255)
        self.pdf.setTitle(self.doc_title)
        self.pdf.drawCentredString(300,690, self.title)

        # PDF_line
        self.pdf.line(30,670,550,670)
        self.pdf.line(30,75,550,75)

        # PDF_Text
        # self.text = pdf.beginText(40,610)
        # self.pdf.setFont('Courier',12)
        # for line in text_line:
        #     text.textLine(line)
        # pdf.drawText(text)

        # PDF_Image
        self.pdf.drawInlineImage('pdf_gen/Images/hbrs.jpg', -70, 770,height=40,preserveAspectRatio=True)
        self.pdf.drawInlineImage('pdf_gen/Images/bit.jpg', -700, 770,height=40,preserveAspectRatio=True)

        # PDF_Table_Properties
        self.data_1     = [[self.test_type,'','','','Results'],
                    ['','','','',''],
                    [self.query_1, '', '', '', self.result_1],
                    [self.query_2, '', '', '', self.result_2],
                    [self.query_3, '', '', '', self.result_3],
                    [self.query_4, '', '', '', self.result_4],
                    [self.query_5, '', '', '', self.result_5],
                    [self.query_6, '', '', '', self.result_6],
                    [self.query_7, '', '', '', self.result_7],
                    [self.query_8, '', '', '', self.result_8],
                    [self.query_9, '', '', '', self.result_9],
                    [self.query_10, '', '', '', self.result_10],
                    [self.query_11, '', '', '', self.result_11],
                    [self.query_12, '', '', '', self.result_12],
                    [self.query_13, '', '', '', self.result_13],
                    [self.query_14, '', '', '', self.result_14],
                    [self.query_15, '', '', '', self.result_15],
                    ]
        ##############################
        self.table_1 = Table(self.data_1, colWidths=[1.3*inch] * 1)
        self.style = TableStyle([
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

        self.border= TableStyle([
            ('BOX',(0,0),(-1,-1),2,(255,255,255)),
            # ('LINEBEFORE',(2,1),(2,-1),2,colors.white), # Horizontal Line
            # ('LINEABOVE',(0,2),(-1,2),2,colors.white), # Veritcal Line
            ('GRID',(0,1),(-1,2),2,colors.white),
            ('GRID',(0,4),(-1,5),2,colors.white),
            ('GRID',(0,7),(-1,8),2,colors.white),
        ])

        self.pdf_header_table(self.table_1,2.4*inch,620,1*inch, 350,'',
                              16,self.border,self.style,0,120,255)
        # add pie chart of passed and failed tests.
        self.pdf.showPage()
        self.pdf.save()