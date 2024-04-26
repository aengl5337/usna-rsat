# -*- coding: utf-8 -*-
"""
Created on Mon Aug  7 19:53:03 2023

dbSaveImagesExample.py

sqlalchemy-media example (from https://pypi.org/project/sqlalchemy-media/)

() - Fix TypeError: type str doesn't define __round__ method
=> x0, y0, x1, y1 = map(int, map(round, box)) needs box to be specified with (left, upper, right, lower)-tuple, not {'left': 0.1, 'top': 0.1, 'width': 0.8, 'height': 0.8}
=> Appears to be compatibility issue, workaround: don't specify crop

() - fix AttributeError: type object 'Image' has no attribute 'load'
=> pixel = Image.Image.load(self) should refer to Image.Image class of PIL, but instead uses sqlAlchemy-media's!
=> Change 'Image' to 'sqlalchemy_media.Image' in this script
=> Didn't work, still getting (when at line 156 of ImageFile.py)
Image
<module 'PIL.Image' from 'C:\\Users\\Alec\\anaconda3\\envs\\tf\\lib\\site-packages\\PIL\\Image.py'>

Image.Image
<class 'sqlalchemy_media.attachments.Image'>



@author: Alec
"""

import functools
import json
from os.path import exists, join

from sqlalchemy import create_engine, TypeDecorator, Unicode, Column, Integer
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base

from sqlalchemy_media import StoreManager, FileSystemStore, ImageAnalyzer, ImageValidator, ImageProcessor
# import sqlalchemy_media.Image as sqlamImage # Doesn't work
import sqlalchemy_media


# TEMP_PATH = '/tmp/sqlalchemy-media'
TEMP_PATH = './'
Base = declarative_base()
engine = create_engine('sqlite:///:memory:', echo=False)
session_factory = sessionmaker(bind=engine)


StoreManager.register(
    'fs',
    functools.partial(FileSystemStore, TEMP_PATH, 'http://static.example.org/'),
    default=True
)


class Json(TypeDecorator):
    impl = Unicode

    def process_bind_param(self, value, engine):
        return json.dumps(value)

    def process_result_value(self, value, engine):
        if value is None:
            return None

        return json.loads(value)


class ProfileImage(sqlalchemy_media.Image):
    __pre_processors__ = [
        ImageAnalyzer(),
        ImageValidator(
            minimum=(80, 80),
            maximum=(800, 600),
            min_aspect_ratio=1.2,
            content_types=['image/jpeg', 'image/png']
        ),
        ImageProcessor(
            fmt='jpeg',
            width=120,
            # crop=dict(                
            #     # left=.1,
            #     # top=.1,
            #     # width=.8,
            #     # height=.8,
            #     left='10%',
            #     top='10%',
            #     width='80%',
            #     height='80%',
            # )
        )
    ]


class Person(Base):
    __tablename__ = 'person'

    id = Column(Integer, primary_key=True)
    name = Column(Unicode(100))
    image = Column(ProfileImage.as_mutable(Json))

    def __repr__(self):
        return "<%s id=%s>" % (self.name, self.id)


Base.metadata.create_all(engine, checkfirst=True)

if __name__ == '__main__':
    session = session_factory()

    with StoreManager(session):
        person1 = Person()
        person1.image = ProfileImage.create_from('https://www.python.org/static/img/python-logo@2x.png')
        session.add(person1)
        session.commit()

        print('Content type:', person1.image.content_type)
        print('Extension:', person1.image.extension)
        print('Length:', person1.image.length)
        print('Original filename:', person1.image.original_filename)

        thumbnail = person1.image.get_thumbnail(width=32, auto_generate=True)
        print(thumbnail.height)
        assert exists(join(TEMP_PATH, thumbnail.path))

        thumbnail = person1.image.get_thumbnail(ratio=.3, auto_generate=True)
        print(thumbnail.width, thumbnail.height)
        assert exists(join(TEMP_PATH, thumbnail.path))

        person1.image.attach('https://www.python.org/static/img/python-logo.png')
        session.commit()

        print('Content type:', person1.image.content_type)
        print('Extension:', person1.image.extension)
        print('Length:', person1.image.length)
        print('Original filename:', person1.image.original_filename)

    with StoreManager(session, delete_orphan=True):
        deleted_filename = join(TEMP_PATH, person1.image.path)
        person1.image = None
        session.commit()

        assert not exists(deleted_filename)

        person1.image = ProfileImage.create_from('https://www.python.org/static/img/python-logo.png')
        session.commit()

        print('Content type:', person1.image.content_type)
        print('Extension:', person1.image.extension)
        print('Length:', person1.image.length)
        print('Original filename:', person1.image.original_filename)
        
# Will produce:

# Content type: image/jpeg
# Extension: .jpg
# Length: 2020
# Original filename: https://www.python.org/static/img/python-logo@2x.png
# 8
# 28 7
# Content type: image/jpeg
# Extension: .jpg
# Length: 2080
# Original filename: https://www.python.org/static/img/python-logo.png
# Content type: image/jpeg
# Extension: .jpg
# Length: 2080
# Original filename: https://www.python.org/static/img/python-logo.png