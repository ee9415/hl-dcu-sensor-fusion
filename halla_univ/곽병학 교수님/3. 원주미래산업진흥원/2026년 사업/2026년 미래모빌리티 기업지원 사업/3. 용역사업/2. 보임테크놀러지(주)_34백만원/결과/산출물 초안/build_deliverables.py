from pathlib import Path
from reportlab.platypus import SimpleDocTemplate, Paragraph, Table, TableStyle, Spacer
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.pagesizes import A4, landscape
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
pdfmetrics.registerFont(TTFont('Malgun','C:/Windows/Fonts/malgun.ttf'))
pdfmetrics.registerFont(TTFont('MalgunBold','C:/Windows/Fonts/malgunbd.ttf'))
rows=[
['1','요구사항·설계','카메라 기능 요구사항 및 인터페이스 정의서','6대 카메라별 역할, 인지 기능, 영상·Depth 입력, 인지결과 출력, 카메라 ID·Timestamp·좌표계, 요구사항별 검증 방법','PDF·편집 원본'],
['2','AI 모델','최종 AI 인지모델 및 설정 파일','객체·차선·정지선·신호등·신호색 기능별 적용 모델 또는 처리 설정, 추론 임계값, 클래스 정의, 모델 버전','모델 파일·설정 파일'],
['3','AI 모델','인지모델 개발 및 최적화 보고서','모델 구성·선정 근거, 데이터 구성·라벨링 기준, 학습 또는 튜닝 내역, 평가 방법과 결과, 적용 한계','PDF·편집 원본'],
['4','Edge 적용','변환 모델 및 카메라 배포 패키지','ONNX·OpenVINO IR·blob 변환 파일, OAK-D Pro PoE 실행 프로그램, 채널별 설정, 의존성·버전·파일 해시 목록','파일 묶음'],
['5','ROS2 연동','거리·연속 프레임 처리 및 ROS2 출력 소프트웨어','객체 거리 산출, 연속 프레임 처리, 카메라 식별정보·Timestamp를 포함한 인지결과 발행, 실행·설정 파일','소스코드·ROS2 패키지'],
['6','ROS2 연동','ROS2 메시지 및 토픽 정의서','메시지 필드·자료형·단위, 토픽명, Frame ID, 발행 조건, QoS, 거리 무효값·미검출·통신 오류 처리','PDF·편집 원본'],
['7','시험·검증','6채널 통합 시험결과보고서','채널별 처리속도, 6대 동시구동·수신율, 객체·차선·정지선·신호등·신호색 기능시험, 거리·연속 프레임 처리, ROS2·DCU 수신 결과','PDF·편집 원본'],
['8','시험·검증','시험 증빙자료','시험영상·화면 캡처, 채널별 송수신 로그, ROS2 기록, 실측 원시데이터, 시험 장비·모델·설정·일시 정보','MP4·PNG·CSV·로그·rosbag2 등'],
['9','운영·인수인계','설치 및 운영 매뉴얼','환경 구성, 카메라 연결·설정, 모델 설치, 실행·종료, 인지결과 확인, 로그 수집, 장애 진단·복구','PDF·편집 원본'],
['10','최종 보고','용역 최종 결과보고서 및 납품 명세서','과업별 수행 결과, 요구사항 대비 달성 현황, 시험 결과 요약, 미완료·제한사항, 납품 파일 경로·버전·목록','PDF·편집 원본']]
body=ParagraphStyle('body',fontName='Malgun',fontSize=9,leading=13,wordWrap='CJK')
bold=ParagraphStyle('bold',parent=body,fontName='MalgunBold')
title=ParagraphStyle('title',fontName='MalgunBold',fontSize=19,leading=26)
sub=ParagraphStyle('sub',parent=body,fontSize=10,leading=15)
p=lambda s,sty=body:Paragraph(s,sty)
out=Path('output/pdf/보임테크놀러지_AI인지SW_산출물목록_초안.pdf')
doc=SimpleDocTemplate(str(out),pagesize=landscape(A4),leftMargin=30,rightMargin=30,topMargin=25,bottomMargin=25,title='보임테크놀러지 AI 인지 소프트웨어 개발 용역 산출물 목록 초안',author='큐센텍')
story=[p('AI 인지 소프트웨어 개발 용역 산출물 목록',title),Spacer(1,6),p('수행사: 보임테크놀러지(주)  |  적용 장비: OAK-D Pro PoE 6대  |  용역금액: 34,000,000원  |  초안 · 2026-11-25',sub),Spacer(1,12)]
data=[[p(s,bold) for s in ['번호','구분','산출물명','포함 내용','제출 형식']]]+[[p(s,bold if j==2 else body) for j,s in enumerate(r)] for r in rows]
t=Table(data,colWidths=[30,72,190,374,116],repeatRows=1,hAlign='LEFT')
t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#DFE8F0')),('VALIGN',(0,0),(-1,-1),'MIDDLE'),('GRID',(0,0),(-1,-1),.5,colors.HexColor('#B8C5D0')),('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#F6F8FA')])]))
story += [t,Spacer(1,9),p('※ 준비해야 할 산출물의 초안이며, 현재 확보 여부와 완성도는 별도 확인합니다. 문서는 최종 결과보고서 본문·부록으로 통합할 수 있으나 모델·소스코드·설정·시험 증빙은 별도 파일로 납품합니다. 각 인지 기능의 모델 또는 후처리 알고리즘 적용 방식은 실제 구현에 맞춰 명시합니다.',ParagraphStyle('note',parent=body,fontSize=8,leading=11))]
doc.build(story)
