<?xml version="1.0" encoding="UTF-8"?>
<!--  ***************************************************************************
*  (c) 2010 by STEMMER IMAGING
*	 Author:  Britta Ladwig-Attinger
*
*  License: This file is published under the license of the EMVA GenICam  Standard Group. 
*  A text file describing the legal terms is included in  your installation as 'GenICam_license.pdf'. 
*  If for some reason you are missing  this file please contact the EMVA or visit the website
*  (http://www.genicam.org) for a full copy.
* 
*  THIS SOFTWARE IS PROVIDED BY THE EMVA GENICAM STANDARD GROUP "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,  
*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  
*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE EMVA GENICAM STANDARD  GROUP 
*  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  SPECIAL, 
*  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT  LIMITED TO, 
*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  DATA, OR PROFITS; 
*  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  THEORY OF LIABILITY, 
*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) 
*  ARISING IN ANY WAY OUT OF THE USE  OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
******************************************************************************** 
  --> 

<xsl:stylesheet version="1.0" xmlns:gen_1_0="http://www.genicam.org/GenApi/Version_1_0" xmlns:gen_1_1="http://www.genicam.org/GenApi/Version_1_1" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<!-- Replace the follwing select value with the first name that shall be extracted, e.g.:
		select="'ChunkDataControl'"/>-->
	<xsl:param name="start" select="'ChunkDataControl'"/>
	<xsl:output method="xml" indent="yes"/>
	<xsl:strip-space elements="*"/>
	<xsl:key name="k1" match="*[@Name]" use="@Name"/>
	<xsl:template match="/">
		<xsl:apply-templates select="gen_1_0:RegisterDescription|gen_1_1:RegisterDescription"/>
	</xsl:template>
	<xsl:template match="gen_1_0:RegisterDescription|gen_1_1:RegisterDescription">
		<xsl:copy>
			<xsl:copy-of select="@*"/>
			<xsl:apply-templates select="key('k1', $start)"/>
		</xsl:copy>
	</xsl:template>
	<xsl:template match="*[@Name]">
		<xsl:param name="processed" select="'|'"/>
		<xsl:param name="current"/>
		<xsl:choose>
			<xsl:when test="local-name()='EnumEntry'">
				<xsl:if test="$current='pEnumEntry'">
					<xsl:copy-of select="."/>
					<xsl:apply-templates select="descendant::*[starts-with(local-name(), 'p')]">
						<xsl:with-param name="processed" select="concat($processed, generate-id(current()), '|')"/>
					</xsl:apply-templates>
				</xsl:if>
			</xsl:when>
			<xsl:when test="name()='StructEntry'">
				<xsl:copy-of select=".."/>
				<xsl:apply-templates select="descendant::*[starts-with(local-name(), 'p')]">
					<xsl:with-param name="processed" select="concat($processed, generate-id(current()), '|')"/>
				</xsl:apply-templates>
			</xsl:when>
			<xsl:when test="name()!='StructEntry'">
				<xsl:copy-of select="."/>
				<xsl:apply-templates select="descendant::*[starts-with(local-name(), 'p')]">
					<xsl:with-param name="processed" select="concat($processed, generate-id(current()), '|')"/>
				</xsl:apply-templates>
			</xsl:when>
		</xsl:choose>
	</xsl:template>
	<xsl:template match="*[@Name]//*[starts-with(local-name(), 'p')]">
		<xsl:param name="processed"/>
		<xsl:apply-templates select="key('k1', .)[not(contains($processed, generate-id()))]">
			<xsl:with-param name="processed" select="$processed"/>
			<xsl:with-param name="current" select="local-name()"/>
		</xsl:apply-templates>
	</xsl:template>
</xsl:stylesheet>
